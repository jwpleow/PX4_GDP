
"use strict";

let CommandTOL = require('./CommandTOL.js')
let ParamSet = require('./ParamSet.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let FileTruncate = require('./FileTruncate.js')
let SetMavFrame = require('./SetMavFrame.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let FileWrite = require('./FileWrite.js')
let WaypointPush = require('./WaypointPush.js')
let CommandHome = require('./CommandHome.js')
let FileRead = require('./FileRead.js')
let FileMakeDir = require('./FileMakeDir.js')
let WaypointPull = require('./WaypointPull.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let FileChecksum = require('./FileChecksum.js')
let FileRemove = require('./FileRemove.js')
let CommandBool = require('./CommandBool.js')
let LogRequestList = require('./LogRequestList.js')
let CommandInt = require('./CommandInt.js')
let LogRequestData = require('./LogRequestData.js')
let FileList = require('./FileList.js')
let FileClose = require('./FileClose.js')
let ParamPush = require('./ParamPush.js')
let ParamGet = require('./ParamGet.js')
let SetMode = require('./SetMode.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let StreamRate = require('./StreamRate.js')
let ParamPull = require('./ParamPull.js')
let MessageInterval = require('./MessageInterval.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let FileRename = require('./FileRename.js')
let WaypointClear = require('./WaypointClear.js')
let CommandLong = require('./CommandLong.js')
let FileOpen = require('./FileOpen.js')

module.exports = {
  CommandTOL: CommandTOL,
  ParamSet: ParamSet,
  WaypointSetCurrent: WaypointSetCurrent,
  FileTruncate: FileTruncate,
  SetMavFrame: SetMavFrame,
  LogRequestEnd: LogRequestEnd,
  VehicleInfoGet: VehicleInfoGet,
  FileWrite: FileWrite,
  WaypointPush: WaypointPush,
  CommandHome: CommandHome,
  FileRead: FileRead,
  FileMakeDir: FileMakeDir,
  WaypointPull: WaypointPull,
  CommandTriggerControl: CommandTriggerControl,
  FileChecksum: FileChecksum,
  FileRemove: FileRemove,
  CommandBool: CommandBool,
  LogRequestList: LogRequestList,
  CommandInt: CommandInt,
  LogRequestData: LogRequestData,
  FileList: FileList,
  FileClose: FileClose,
  ParamPush: ParamPush,
  ParamGet: ParamGet,
  SetMode: SetMode,
  CommandTriggerInterval: CommandTriggerInterval,
  StreamRate: StreamRate,
  ParamPull: ParamPull,
  MessageInterval: MessageInterval,
  FileRemoveDir: FileRemoveDir,
  FileRename: FileRename,
  WaypointClear: WaypointClear,
  CommandLong: CommandLong,
  FileOpen: FileOpen,
};
