class Device:

    def __init__(self, uid):
        self.uid = uid

    def __eq__(self, other):
        if (isinstance(other, Device)):
            return self.uid == other.uid
        return False

    def __str__(self):
        return "uid: " + self.uid + "\n" + "Last Data: " + json.dumps(self.device_data)

    def update_data(self, websocket, device_data):
        self.websocket = websocket
        self.device_data = device_data

class Devices:

    def __init__(self):
        self.devices = []

    def __str__(self):
        for device in self.devices:
            print("Device: " + str(device))
        return "Number of Devices: " + str(len(self.devices))

    def get_device(self, uid):
        for device in self.devices:
            if device.uid == uid:
                return device
        return None

    def get_device_by_websocket(self, websocket):
        for device in self.devices:
            if device.websocket == websocket:
                return device
        return None

    def remove_device_by_socket(self, websocket):
        device = self.get_device_by_websocket(websocket)
        if device is not None:
            print("remove_device_by_socket found: " + device.uid)
            index = None
            for i in range(len(self.devices)):
                if self.devices[i].uid == device.uid:
                    index = i
            self.devices.pop(index)

            device_data = get_device_data(device.uid)
            device_data.pop("host", None)
            save_device_data(device.uid, device_data) # {"ssh_host": None, "camera_host": None, "control_host": None}

    async def register(self, websocket, device_uid, data):
        if "ssh_host" not in data or "camera_host" not in data or "control_host" not in data:
            return {"success": False, "error": {"code": 1002, "message": "Incorrect parameters"}}

        device_data = get_device_data(device_uid)
        device_data["host"] = {"ssh_host": data["ssh_host"], "camera_host": data["camera_host"], "control_host": data["control_host"]}

        device = self.get_device(device_uid)
        if device is None:
            device = Device(device_uid)
            self.devices.append(device)
        device.update_data(websocket, device_data)

        save_device_data(device_uid, device_data)

        return {"success": True, "data": device_data}

    async def save_device_params(self, websocket, device_uid, data):
        device_data = get_device_data(device_uid)
        params = device_data["data"] if "data" in device_data else {}
        for key in data:
            if key != "device_uid":
                params[key] = data[key]
        device_data["data"] = params
        save_device_data(device_uid, device_data)
        return {"success": True, "data": device_data}

def save_device_data(device_uid, data):
    json_array = {}
    with open('data.json', 'r') as json_file:
        json_array = json.load(json_file)
        json_file.close()
    json_array[device_uid] = data;
    with open('data.json', "w") as json_file:
        json_file.write(json.dumps(json_array))
        json_file.close()

def get_device_data(device_uid):
    json_array = {}
    with open('data.json', 'r') as json_file:
        json_array = json.load(json_file)
        json_file.close()
    return json_array[device_uid] if device_uid in json_array else {}

import asyncio
import json
import websockets
import ssl

DEVICES = Devices()

async def send_message_to_socket(websocket, request, response):
    request["response"] = response
    print("SEND DATA: " + json.dumps(request))
    await websocket.send(json.dumps(request))

async def request_data_from_user_to_device(websocket, request):
    action = request["action"]
    data = request["data"] if "data" in request else None
    device_uid = data["device_uid"] if "device_uid" in data else None
    if device_uid is None:
        await send_message_to_socket(websocket, request, {"success": False, "error": {"code": 1002, "message": "Incorrect parameters"}})
    else:
        if action == "register":
            response = await DEVICES.register(websocket, device_uid, data)
            await send_message_to_socket(websocket, request, response)
            print("-----REGISTER-----")
            print(DEVICES)
            print("------------------")
        elif action == "save_device_params":
            response = await DEVICES.save_device_params(websocket, device_uid, data)
            await send_message_to_socket(websocket, request, response)
        else:
            await send_message_to_socket(websocket, request, {"success": False, "error": {"code": 1000, "message": "Incorrect action request"}})

async def unregister(websocket):
    DEVICES.remove_device_by_socket(websocket)
    print("----UNREGISTER----")
    print(DEVICES)
    print("------------------")

async def counter(websocket, path):
    try:
        async for message in websocket:
            dictionary = json.loads(message)
            # if "action" in dictionary and dictionary["action"] != "device_data":
            print("RECEIVE DATA: " + message)
            if "action" in dictionary:
                await request_data_from_user_to_device(websocket, dictionary)
            else:
                await send_message_to_socket(websocket, dictionary, {"success": False, "error": {"code": 1000, "message": "Incorrect action request"}})
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        await unregister(websocket)

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain("/root/cert.pem", "/root/cert.pem")
start_server = websockets.serve(counter, "0.0.0.0", 8443, max_size=9000000, ssl=ssl_context)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()