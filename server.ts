// Production server for the built app (`bun run start`).
//
// Serves the static build in dist/. Reads PORT from the environment (Railway
// injects it) and binds 0.0.0.0 so the container is reachable. Content-hashed
// assets under /assets/ are cached immutably; everything else is no-cache.

import { join, normalize } from "node:path";

const DIST = join(import.meta.dir, "dist");
const PORT = Number(process.env.PORT ?? 3000);

async function serveStatic(pathname: string): Promise<Response> {
  let rel = decodeURIComponent(pathname);
  if (rel === "/" || rel === "") rel = "/index.html";

  const filePath = normalize(join(DIST, rel));
  // Reject path traversal outside dist/.
  if (filePath !== DIST && !filePath.startsWith(DIST + "/")) {
    return new Response("Forbidden", { status: 403 });
  }

  const file = Bun.file(filePath);
  if (!(await file.exists())) return new Response("Not found", { status: 404 });

  const cacheControl = rel.startsWith("/assets/")
    ? "public, max-age=31536000, immutable" // hashed filenames
    : "no-cache";
  return new Response(file, { headers: { "cache-control": cacheControl } });
}

const server = Bun.serve({
  port: PORT,
  hostname: "0.0.0.0",
  fetch(req) {
    const url = new URL(req.url);
    return serveStatic(url.pathname);
  },
});

console.log(`Serving dist/ on http://0.0.0.0:${server.port}`);
