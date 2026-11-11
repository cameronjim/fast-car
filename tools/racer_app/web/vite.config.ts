import preact from "@preact/preset-vite";
import { defineConfig } from "vitest/config";

// The agent serves dist/ from its own origin, so relative asset paths are enough.
export default defineConfig({
  base: "./",
  plugins: [preact()],
  build: { outDir: "dist", sourcemap: false, target: "es2020" },
  server: {
    port: 5173,
    // `npm run dev` proxies the agent API so the UI can be developed against a running car.
    proxy: { "/api": "http://racer-car.local:8080", "/health": "http://racer-car.local:8080" },
  },
  test: { environment: "node", include: ["src/**/*.test.ts"] },
});
