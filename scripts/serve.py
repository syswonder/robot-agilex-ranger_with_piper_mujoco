#!/usr/bin/env python3
"""Serve mutable source without caching while caching large runtime assets."""

import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class DevelopmentRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        extension = Path(self.path.split("?", 1)[0]).suffix.lower()
        if extension in {".obj", ".png", ".stl", ".spz", ".wasm", ".onnx"}:
            self.send_header("Cache-Control", "public, max-age=86400")
        else:
            self.send_header("Cache-Control", "no-cache, must-revalidate")
        super().end_headers()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5173)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.bind, args.port), DevelopmentRequestHandler)
    print(f"Serving MuJoCo Robonix on http://{args.bind}:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
