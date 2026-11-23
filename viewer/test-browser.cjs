const assert = require('node:assert/strict');

async function smokeTest() {
  const target = process.env.SITE_URL;
  if (!target) throw new Error('SITE_URL is required');

  let chromium;
  try {
    ({ chromium } = require('playwright-core'));
  } catch (error) {
    throw new Error('Install the Playwright dependency before running this test');
  }

  const browser = await chromium.launch();
  try {
    const page = await browser.newPage({ viewport: { width: 1280, height: 800 } });
    await page.goto(target, { waitUntil: 'networkidle' });
    await page.getByText('Improvement / v2 avoids collision').waitFor();
    assert.equal(await page.locator('.run').count(), 2);

    await page.getByRole('button', { name: 'Pedestrian crossing' }).click();
    await page.getByText('Regression / v2 collides').waitFor();
    assert.equal(await page.locator('.run').count(), 2);

    await page.setViewportSize({ width: 390, height: 844 });
    const layout = await page.evaluate(() => ({
      overflow: document.documentElement.scrollWidth > innerWidth,
      panels: document.querySelectorAll('.run').length,
    }));
    assert.equal(layout.overflow, false);
    assert.equal(layout.panels, 2);
  } finally {
    await browser.close();
  }
}

smokeTest().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
