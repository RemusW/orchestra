import koaProtobuf from 'koa-protobuf';
import Router from 'koa-router';

import logger from './common/logger';
import { telemetry } from './messages';

let router = new Router();

// Encode outbound protobuf messages.
router.use(koaProtobuf.protobufSender());

// Middleware for handling timeout errors. Enabled on some endpoints.
let timeout = async (ctx, next) => {
  try {
    await next();
  } catch (err) {
    logger.error(err.message);
    ctx.status = 504;
  }
};

// Middleware to parse a request mission body.
let mission = koaProtobuf.protobufParser(telemetry.RawMission);

// Middleware to parse a request mission current body.
let missionCurrent = koaProtobuf.protobufParser(telemetry.MissionCurrent);

router.get('/api/alive', (ctx) => {
  ctx.body = 'Yes, I\'m alive!\n';
});

router.get('/api/interop-telem', (ctx) => {
  ctx.proto = ctx.plane.getInteropTelem();
});

router.get('/api/camera-telem', (ctx) => {
  ctx.proto = ctx.plane.getCameraTelem();
});

router.get('/api/overview', (ctx) => {
  ctx.proto = ctx.plane.getOverview();
});

router.get('/api/raw-mission', timeout, async (ctx) => {
  ctx.proto = await ctx.plane.getRawMission();
});

router.get('/autopilot-mode', async (ctx) => {
  ctx.proto = ctx.plane.getAutopilotMode();
})

router.post('/api/raw-mission', mission, timeout, async (ctx) => {
  await ctx.plane.setRawMission(ctx.request.proto);
  ctx.status = 200;
});

router.get('/api/mission-current', timeout, async (ctx) => {
  ctx.proto = await ctx.plane.getMissionCurrent();
});

router.post('/api/mission-current', missionCurrent, timeout, async (ctx) => {
  await ctx.plane.setMissionCurrent(ctx.request.proto);
  ctx.status = 200;
});

export default router;
