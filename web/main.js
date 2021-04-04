#!/usr/bin/env node

const port_input_udp_msgpack = 44333;
const port_output_http = 8005;

const dgram = require('dgram');
const server = dgram.createSocket('udp4');
const msgpack = require("@msgpack/msgpack");

const express = require('express');
const app = express();
const http = require('http').Server(app);
const io = require('socket.io')(http);

/*** UDP ***/

server.on('error', (err) => {
  console.log(`server error:\n${err.stack}`);
  server.close();
});

server.on('message', (msg, rinfo) => {
  //console.log(`server got (length: ${msg.length}): ${msg} from ${rinfo.address}:${rinfo.port}`);

  const msg_uint8array = new Uint8Array(msg.buffer, msg.byteOffset, msg.length / Uint8Array.BYTES_PER_ELEMENT);

  const msg_decoded = msgpack.decode(msg_uint8array);

  //console.log(`decoded: ${JSON.stringify(msg_decoded, null, 4)}`);

  io.emit('update', msg_decoded);
});

server.on('listening', () => {
  const address = server.address();
  console.log(`UDP server listening ${address.address}:${address.port}`);
});

server.bind(port_input_udp_msgpack);

/*** HTTP ***/

app.use(express.static('htdocs'));

http.listen(port_output_http, () => {
  console.log(`HTTP listening on *:${port_output_http} dev link: http://127.0.0.1:${port_output_http}`);
});