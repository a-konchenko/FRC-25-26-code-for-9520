import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import { deployFiles } from './server/deployFiles';

export default defineConfig({
  plugins: [react(), deployFiles()],
  server: { port: 5173, strictPort: true },
});
