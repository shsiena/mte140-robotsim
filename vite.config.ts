import { defineConfig } from "vite";

export default defineConfig({
  base: "./", // relative asset paths so the build can be served from any path
  server: { 
    port: 5173, 
    allowedHosts: ["arch"],
  },
  build: {
    outDir: "dist",
    target: "esnext",
    sourcemap: "hidden", // maps emitted but not referenced from the bundles
    minify: true,
  },
});
