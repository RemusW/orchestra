import queue from 'async/queue';

import { interop, telemetry } from './messages';

import MavlinkSocket from './mavlink-socket';
import { receiveMission, sendMission, sendMissionCurrent } from './mission';
import { degrees, modDegrees360, modDegrees180 } from './util';

const ConnectionState = Object.freeze({
  NOT_CONNECTED: Symbol('not_connected'),
  IDLE:          Symbol('idle'),
  READING:       Symbol('reading'),
  WRITING:       Symbol('writing')
});

/** Holds the state of a plane over a MAVLink UDP connection. */
export default class Plane {
  /**
   * Create a new Plane instance.
   *
   * @param {Object} options
   * @param {string} options.host
   * @param {number} options.port
   */
  constructor(options) {
    this._mav = new MavlinkSocket(options.host, options.port);
    this._cxnState = ConnectionState.NOT_CONNECTED;

    // Async transaction queue. This only allows one transaction to
    // be active at a time.
    this._taskQueue = queue(async (asyncTask) => {
      return await asyncTask();
    }, 1);

    this._overview = telemetry.Overview.create({
      time: 0,
      pos: {},
      rot: {},
      alt: {},
      vel: {},
      speed: {},
      battery: {},
      mode: 0
    });
    // Will be assigned on each GLOBAL_POSITION_INT message.
    this._interopTelem = null;
    this._cameraTelem = telemetry.CameraTelem.create({});
    // Will be assigned after mission current or first mission is
    // received.
    this._missionCurrent = null;

    this._bindMessages();
  }

  /** Establish a connection to the plane. */
  async connect() {
    await this._mav.connect();
    await this._startTelemetry();

    this._cxnState = ConnectionState.IDLE;
  }

  /** Execute remaining tasks and close the connection. */
  async disconnect() {
    // Kill the telemetry data stream interval if it is still going.
    clearInterval(this._streamInt);

    await this._waitForTasks();
    await this._mav.close();
  }

  /** Get the overview telemetry. */
  getOverview() {
    return this._overview;
  }

  /** Get the camera telemetry. */
  getCameraTelem() {
    return this._cameraTelem;
  }

  /** Get the interop telemetry. */
  getInteropTelem() {
    return this._interopTelem;
  }

  /**
   * Get the raw mission from the plane.
   *
   * @returns {Promise<telemetry.RawMission>}
   */
  async getRawMission() {
    return await this._execTransaction(async () => {
      this._cxnState = ConnectionState.READING;

      try {
        return await receiveMission(this._mav);
      } finally {
        this._cxnState = ConnectionState.IDLE;
      }
    });
  }

  /**
   * Send a raw mission to the plane.
   *
   * @param {telemetry.RawMission} mission
   * @returns {Promise}
   */
  async setRawMission(mission) {
    await this._execTransaction(async () => {
      this._cxnState = ConnectionState.WRITING;

      try {
        await sendMission(this._mav, mission);
      } finally {
        this._cxnState = ConnectionState.IDLE;
      }
    });
  }

  /**
   * Get the current mission item.
   *
   * If the mission item has already be received. It is loaded from a
   * cached value.
   *
   * @returns {Promise<telemetry.MissionCurrent>}
   */
  async getMissionCurrent() {
    // If the current mission item hasn't been set, then get the
    // mission. The mission item may be set as a side effect.
    if (this._missionCurrent === null) {
      await this.getMission();

      // If mission current wasn't set since there was no mission
      // item marked as current. The mission current is assumed to be
      // zero.
      if (this._missionCurrent === null) {
        this._missionCurrent = telemetry.MissionCurrent.create({
          time: Date.now() / 1000,
          item_number: 0
        });
      }
    }

    return this._missionCurrent;
  }

  /**
   * Set the current mission item.
   *
   * @param {telemetry.MissionCurrent} missionCurrent
   * @returns {Promise}
   */
  async setMissionCurrent(missionCurrent) {
    await this._execTransaction(async () => {
      this._cxnState = ConnectionState.WRITING;

      try {
        await sendMissionCurrent(this._mav, missionCurrent);
      } finally {
        this._cxnState = ConnectionState.IDLE;
      }
    });
  }

  // Add an async task to the queue.
  async _execTransaction(asyncTask) {
    return await new Promise((resolve, reject) => {
      this._taskQueue.push(asyncTask, (err, result) => {
        if (err) reject(err);
        else resolve(result);
      });
    });
  }

  // Send REQUEST_DATA_STREAM messages until a position arrives.
  async _startTelemetry() {
    this._requestDataStream();
    this._streamInt = setInterval(() => this._requestDataStream(), 500);

    return await new Promise((resolve) => {
      // GLOBAL_POSITION_INT is required for interop telemetry and so
      // wait for at least one before considering the service up.
      // This ensures interop telemetry is always valid.
      this._mav.once('GLOBAL_POSITION_INT', () => {
        clearInterval(this._streamInt);
        resolve();
      });
    });
  }

  async _requestDataStream() {
    await this._mav.send('REQUEST_DATA_STREAM', {
      target_system: 1,
      target_component: 1,
      req_stream_id: 0,
      req_message_rate: 5,
      start_stop: 1
    });
  }

  // Wait until all tasks in the queue have been completed. Since
  // this is primarily used in tests, it's best to let all tasks
  // complete. At runtime in the container docker sends SIGKILL so a
  // graceful shutdown wouldn't happen anyways.
  async _waitForTasks() {
    if (!this._taskQueue.idle()) {
      await new Promise(resolve => this._taskQueue.drain = resolve);
    }
  }

  // Attach listeners below to update the state on incomming
  // messages.
  _bindMessages() {
    this._mav
      .on('ATTITUDE', this._onAttitude.bind(this))
      .on('GLOBAL_POSITION_INT', this._onGlobalPositionInt.bind(this))
      .on('MISSION_CURRENT', this._onMissionCurrent.bind(this))
      .on('MISSION_ITEM', this._onMissionItem.bind(this))
      .on('VFR_HUD', this._onVfrHud.bind(this))
      .on('SYS_STATUS', this._onSysStatus.bind(this))
      .on('HEARTBEAT', this._onHeartbeat.bind(this));
  }

  async _onAttitude(fields) {
    let ov = this._overview;
    let ca = this._cameraTelem;

    ov.time = Date.now() / 1000;
    ov.rot.yaw = modDegrees360(degrees(fields.yaw));
    ov.rot.pitch = modDegrees180(degrees(fields.pitch));
    ov.rot.roll = modDegrees180(degrees(fields.roll));

    // Assuming camera is pointed straight down. The roll direction
    // is opposite of the plane, and the pitch is offset by 90
    // degrees.
    ca.time = ov.time;
    ca.yaw = ov.rot.yaw;
    ca.pitch = modDegrees180(degrees(fields.pitch) - 90);
    ca.roll = modDegrees180(-degrees(fields.roll));
  }

  async _onGlobalPositionInt(fields) {
    let ov = this._overview;
    let ca = this._cameraTelem;

    ov.time = Date.now() / 1000;
    ov.pos.lat = modDegrees180(fields.lat / 1e7);
    ov.pos.lon = modDegrees180(fields.lon / 1e7);
    ov.alt.msl = fields.alt / 1000;
    ov.alt.agl = fields.relative_alt / 1000;
    ov.rot.yaw = modDegrees360(fields.hdg / 100);
    ov.vel.x = fields.vx / 100;
    ov.vel.y = fields.vy / 100;
    ov.vel.z = fields.vz / 100;

    ca.time = ov.time;
    ca.lat = ov.pos.lat;
    ca.lon = ov.pos.lon;
    ca.alt = ov.alt.agl;
    ca.yaw = ov.rot.yaw;

    this._interopTelem = interop.InteropTelem.create({
      time: ov.time,
      pos: { lat: ov.pos.lat, lon: ov.pos.lon, alt_msl: ov.alt.msl },
      yaw: ov.rot.yaw
    });
  }

  async _onMissionCurrent(fields) {
    this._missionCurrent = telemetry.MissionCurrent.create({
      time: Date.now() / 1000,
      item_number: fields.seq
    });
  }

  async _onMissionItem(fields) {
    if (fields.current) {
      this._missionCurrent = telemetry.MissionCurrent.create({
        time: Date.now() / 1000,
        item_number: fields.seq
      });
    }
  }

  async _onVfrHud(fields) {
    let ov = this._overview;

    ov.time = Date.now() / 1000;
    ov.speed.airspeed = fields.airspeed;
    ov.speed.ground_speed = fields.groundspeed;
    ov.alt.msl = fields.alt;
  }

  async _onSysStatus(fields) {
    let ov = this._overview;

    ov.time = Date.now() / 1000;
    ov.battery.voltage = fields.voltage_battery / 1000;
    ov.battery.current = fields.current_battery / 100;
    ov.battery.percentage = fields.battery_remaining;
  }

  async _onHeartbeat(fields) {
    let ov = this._overview;

    ov.mode = fields.base_mode;
  }
}
