package.cpath = './build/?.so;' .. package.cpath;
local DspPacket = require('DspPacket');

DspPacket.enter_entry();
DspPacket.dsp_thread();
DspPacket.dsp_exit();
