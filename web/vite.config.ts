import { defineConfig } from 'vitest/config';

import react from '@vitejs/plugin-react';
import tailwindcss from '@tailwindcss/vite';

import path from 'node:path';

export default defineConfig({
  plugins: [react(), tailwindcss()],
  resolve: {
    alias: {
      '@': path.resolve(import.meta.dirname, './src'),
      '@assets': path.resolve(import.meta.dirname, '../assets'),
    },
  },
  build: {
    outDir: 'dist',
    emptyOutDir: true,
    assetsInlineLimit: 0,
    chunkSizeWarningLimit: 1200,
  },
  server: {
    proxy: { '/ws': { target: 'ws://127.0.0.1:8000', ws: true } },
    fs: { allow: ['..'] },
  },
  test: {
    environment: 'node',
    include: ['tests/**/*.test.ts'],
  },
});
