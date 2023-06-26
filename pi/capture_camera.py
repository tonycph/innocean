import asyncio
import websockets
import av
import PIL

# Feed in your raw bytes from socket
def decode_image(raw_bytes: bytes):
    code_ctx = av.CodecContext.create("h264", "r")
    packets = code_ctx.parse(raw_bytes)
    for i, packet in enumerate(packets):
        frames = code_ctx.decode(packet)
        if frames:
            return frames[0].to_image()

async def save_img_from_streaming():

    uri = "ws://127.0.0.1:8080"
    async with websockets.connect(uri) as websocket:
        image_init = await websocket.recv()

        count = 0
        combined = b''

        while count < 2:
            response = await websocket.recv()
            combined += response
            count += 1

        img = decode_image(combined)
        img.save("img1.jpg","JPEG")

asyncio.get_event_loop().run_until_complete(save_img_from_streaming())



