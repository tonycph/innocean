<?php

function get_device_host($device_id, $device_api_key) {
	$json_str = file_get_contents("./data.json");
	$json_arr = json_decode($json_str, true);
	foreach ($json_arr as $id => $data)
		if (strcmp($id, $device_id) == 0) {
			$host = array_key_exists("host", $data) ? $data["host"] : null;
			if (!is_null($host) 
				&& array_key_exists("camera_host", $host) && !is_null($host["camera_host"])
				&& array_key_exists("control_host", $host) && !is_null($host["control_host"])) {
				unset($host["ssh_host"]);
				return array("success" => true, "data" => $host);
			} else {
				return array("success" => false, "error" => array("code" => 1002, "message" => "Device is not activited."));
			}
		}
	return array("success" => false, "error" => array("code" => 1003, "message" => "Device cannot be found."));
}

$postArray = $_POST;
if (count($postArray) == 0)
	$postArray = json_decode(file_get_contents('php://input'), true);

$action = array_key_exists("action", $postArray) ? $postArray["action"] : null;
$data = array_key_exists("data", $postArray) ? $postArray["data"] : null;
$device_id = !is_null($data) && array_key_exists("device_id", $data) ? $data["device_id"] : null;
$device_api_key = !is_null($data) && array_key_exists("device_api_key", $data) ? $data["device_api_key"] : null;

$response = array("success" => false, "error" => array("code" => 1001, "message" => "Parameter is not valid. [action]"));
if (!is_null($action) && strcmp("get_device_host", $action) == 0) {
	if (is_null($device_id) || is_null($device_api_key)) {
		$response = array("success" => false, "error" => array("code" => 1001, "message" => "Parameter is not valid. [device_id/device_api_key]"));
	} else {
		$response = get_device_host($device_id, $device_api_key);
	}
}
echo json_encode($response);
