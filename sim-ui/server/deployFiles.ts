// Vite dev-server plugin: read/write PathPlanner files in the robot project.
//
//   GET    /api/pp/list                  → { paths, autos, settings } with mtimes
//   GET    /api/pp/file?path=<rel>       → file text
//   PUT    /api/pp/file?path=<rel>       → write file (body = JSON text)
//   DELETE /api/pp/file?path=<rel>       → delete file (a backup is kept)
//
// <rel> is relative to <robot project>/src/main/deploy/pathplanner and must be
// paths/<name>.path, autos/<name>.auto or settings.json — nothing else.
// Before a file is first overwritten or deleted in a session, a copy goes to
// sim-ui/.sim-backup/ (outside the deploy folder, so it is never deployed).
import type { Plugin } from 'vite';
import type { IncomingMessage, ServerResponse } from 'node:http';
import { existsSync, mkdirSync, promises as fs } from 'node:fs';
import { basename, dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const ALLOWED = /^(paths\/[^/\\]+\.path|autos\/[^/\\]+\.auto|settings\.json)$/;

export function deployFiles(): Plugin {
  const robotDir = resolve(process.env.ROBOT_PROJECT_DIR ?? join(here, '../../Swerve9520 ctre tuner x project (og home base ver.)'));
  const ppDir = join(robotDir, 'src/main/deploy/pathplanner');
  const backupDir = join(here, '../.sim-backup');
  const backedUp = new Set<string>();

  const resolveRel = (rel: string | null): string | null => {
    if (!rel || rel.includes('..') || !ALLOWED.test(rel)) return null;
    return join(ppDir, rel);
  };

  async function backup(rel: string, abs: string) {
    if (backedUp.has(rel) || !existsSync(abs)) return;
    mkdirSync(backupDir, { recursive: true });
    const stamp = new Date().toISOString().replace(/[:.]/g, '-');
    await fs.copyFile(abs, join(backupDir, `${basename(abs)}.${stamp}`));
    backedUp.add(rel);
  }

  async function listDir(sub: string, ext: string) {
    try {
      const names = await fs.readdir(join(ppDir, sub));
      const out = [];
      for (const n of names) {
        if (!n.endsWith(ext) || n.startsWith('.')) continue;
        const st = await fs.stat(join(ppDir, sub, n));
        out.push({ name: n.slice(0, -ext.length), mtime: st.mtimeMs });
      }
      return out.sort((a, b) => a.name.localeCompare(b.name, undefined, { sensitivity: 'base' }));
    } catch {
      return [];
    }
  }

  async function handle(req: IncomingMessage, res: ServerResponse) {
    const url = new URL(req.url ?? '', 'http://localhost');
    const send = (code: number, body: unknown, type = 'application/json') => {
      res.statusCode = code;
      res.setHeader('Content-Type', type);
      res.end(typeof body === 'string' ? body : JSON.stringify(body));
    };

    if (url.pathname === '/api/pp/list' && req.method === 'GET') {
      let settings: number | null = null;
      try { settings = (await fs.stat(join(ppDir, 'settings.json'))).mtimeMs; } catch { /* none */ }
      return send(200, { root: ppDir, paths: await listDir('paths', '.path'), autos: await listDir('autos', '.auto'), settings });
    }

    if (url.pathname !== '/api/pp/file') return send(404, { error: 'not found' });
    const rel = url.searchParams.get('path');
    const abs = resolveRel(rel);
    if (!abs || !rel) return send(400, { error: `path not allowed: ${rel}` });

    if (req.method === 'GET') {
      try { return send(200, await fs.readFile(abs, 'utf8'), 'application/json'); }
      catch { return send(404, { error: `no such file: ${rel}` }); }
    }

    if (req.method === 'PUT') {
      const chunks: Buffer[] = [];
      for await (const c of req) chunks.push(c as Buffer);
      const text = Buffer.concat(chunks).toString('utf8');
      try { JSON.parse(text); } catch { return send(400, { error: 'body is not valid JSON' }); }
      await backup(rel, abs);
      mkdirSync(dirname(abs), { recursive: true });
      const tmp = `${abs}.tmp-${process.pid}`;
      await fs.writeFile(tmp, text, 'utf8');
      await fs.rename(tmp, abs);
      const st = await fs.stat(abs);
      return send(200, { ok: true, mtime: st.mtimeMs });
    }

    if (req.method === 'DELETE') {
      if (!existsSync(abs)) return send(404, { error: `no such file: ${rel}` });
      await backup(rel, abs);
      await fs.unlink(abs);
      return send(200, { ok: true });
    }

    return send(405, { error: 'method not allowed' });
  }

  return {
    name: 'pathplanner-deploy-files',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        if (!req.url?.startsWith('/api/pp/')) return next();
        handle(req, res).catch((e) => {
          res.statusCode = 500;
          res.end(JSON.stringify({ error: String(e) }));
        });
      });
      console.log(`  PathPlanner files: ${ppDir}`);
    },
  };
}
