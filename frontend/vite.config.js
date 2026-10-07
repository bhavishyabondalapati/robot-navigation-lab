import react from '@vitejs/plugin-react'
import { defineConfig } from 'vite'

// During development, Vite forwards API and WebSocket calls to FastAPI on :8000,
// so the browser only ever talks to one origin (no CORS setup needed).
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api': 'http://127.0.0.1:8000',
      '/ws': { target: 'ws://127.0.0.1:8000', ws: true },
    },
  },
})
