@page dwx_ipc_socketclientserver_sample Socket Inter-Process Communication Sample

The socket inter-process communication sample demonstrates
simple IPC functionalities using network sockets.

## Running the Socket Client / Server Sample

Sample command line:

- Server Process: `./sample_ipc_socketclientserver --role=server --port=49252`
- Client Process: `./sample_ipc_socketclientserver --role=client --port=49252 --ip=127.0.0.1`

Arguments
- `--role` is either "client" or "server" (required).
- `--port` is the port the server will listen on / the client will connect to (optional).
- `--ip` is the server IP the client connects to (optional).

## Output

In the sample the client generates random values, sends them to the server, who echoes them back.
