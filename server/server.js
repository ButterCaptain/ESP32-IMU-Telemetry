const { SerialPort } = require('serialport');
const WebSocket = require('ws');

// Initialize Serial
const port = new SerialPort({ 
    path: '/dev/cu.usbserial-0001', 
    baudRate: 115200 
});

const wss = new WebSocket.Server({ port: 8080 });

let buffer = Buffer.alloc(0);
const PACKET_SIZE = 18; // 2 (Header) + 16 (4 Floats)

port.on('data', (chunk) => {
    // Add the new chunk of bytes to our "pile"
    buffer = Buffer.concat([buffer, chunk]);

    // We run a loop just in case we received multiple packets in one chunk
    while (buffer.length >= PACKET_SIZE) {
        
        // Look for our specific 0xAA 0xBB Header
        if (buffer[0] === 0xAA && buffer[1] === 0xBB) {
            
            // We found a valid start! Extract exactly 22 bytes.
            const packet = buffer.subarray(0, PACKET_SIZE);
            
            // BROADCAST: Send this clean 22-byte binary packet to the browser
            wss.clients.forEach((client) => {
                if (client.readyState === WebSocket.OPEN) {
                    client.send(packet); 
                }
            });

            // Remove the processed packet from the start of the buffer
            buffer = buffer.subarray(PACKET_SIZE);
            
        } else {
            // SYNC PROTECTION: If we didn't find the header at the start, 
            // the data is "shifted." Throw away 1 byte and check again.
            buffer = buffer.subarray(1);
        }
    }
});

wss.on('connection', (ws) => {
    console.log('--- Telemetry Dashboard Connected ---');
});

console.log("Binary bridge active on ws://localhost:8080");
