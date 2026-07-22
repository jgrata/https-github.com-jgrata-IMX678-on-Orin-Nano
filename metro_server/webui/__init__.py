"""Server-side web UI for the IMX678/Jetson camera (skeleton).

An additive front-end that runs on the Jetson and talks to the existing
image_server.py over localhost:9000 as one more client (image_server accepts
concurrent connections). It does NOT modify image_server or the wire protocol,
so the MATLAB client and the PC image-data path keep working unchanged.
Portable to the IQ9: only camera_client.py knows the transport.
"""
