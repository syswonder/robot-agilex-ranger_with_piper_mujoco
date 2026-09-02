import { chromium } from 'playwright';

const visible = process.env.SIM_HEADLESS !== '1';
const url = process.env.SIM_URL || 'http://127.0.0.1:5180/?environment=scenesmith_house_187';
const browser = await chromium.launch({ headless: !visible });
const context = await browser.newContext({ viewport: { width: 1440, height: 900 } });
const page = await context.newPage();
page.on('console', (message) => console.log(`[browser:${message.type()}] ${message.text()}`));
page.on('pageerror', (error) => console.error(`[browser:error] ${error.stack || error}`));
await page.goto(url, { waitUntil: 'domcontentloaded', timeout: 120_000 });
await page.waitForFunction(
  () => window.mujocoApp?.getState?.().model && window.mujocoBridge?.status?.().connected,
  null,
  { timeout: 180_000 },
);
console.log(`MuJoCo browser is ready at ${url}`);
process.send?.({ type: 'ready', url });

const stop = async () => {
  await browser.close();
  process.exit(0);
};
process.on('SIGINT', stop);
process.on('SIGTERM', stop);
await new Promise(() => {});
