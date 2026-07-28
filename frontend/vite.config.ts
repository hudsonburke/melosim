import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

const API_PORT = process.env.MELSIM_API_PORT || '3000';

export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/scene': `http://localhost:${API_PORT}`,
      '/import': `http://localhost:${API_PORT}`,
      '/meshes': `http://localhost:${API_PORT}`,
      '/attach_mesh': `http://localhost:${API_PORT}`,
      '/attach_body': `http://localhost:${API_PORT}`,
      '/body_builder': `http://localhost:${API_PORT}`,
    },
  },
})
