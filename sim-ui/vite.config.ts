import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import { deployFiles } from './server/deployFiles.ts';

export default defineConfig({
  plugins: [react(), deployFiles()],
  server: { port: 5173, strictPort: true },
});
