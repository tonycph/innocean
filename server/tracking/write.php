<?php

// header('Access-Control-Allow-Origin: *');
// $_GET = array("filename" => "index_data.json", "lang" => "en", "action" => "hello");

$filename = null;
$params = array();

foreach ($_GET as $key => $value) {
	if (strcmp($key, "filename") == 0) {
		$filename = $value;
	} else {
		$params[$key] = is_null($value) || strcmp($value, "null") == 0 ? "" : htmlspecialchars_decode($value);
	}
}

if (!array_key_exists("ip", $params)) {
	if (!empty($_SERVER['HTTP_CLIENT_IP'])) {
	    $params["ip"] = $_SERVER['HTTP_CLIENT_IP'];
	} elseif (!empty($_SERVER['HTTP_X_FORWARDED_FOR'])) {
	    $params["ip"] = $_SERVER['HTTP_X_FORWARDED_FOR'];
	} else {
	    $params["ip"] = $_SERVER['REMOTE_ADDR'];
	}
}

$params["createdAt"] = (new DateTime("now", new DateTimeZone('Asia/Shanghai')))->format('Y-m-d\TH:i:s\.v\Z');

if (!is_null($filename) && strcmp($filename, "") != 0 && count($params) > 0) {
	$oldData = [];
	if (file_exists($filename)) {
		$content = file_get_contents($filename);
		$oldData = json_decode($content, true);
		$oldData[] = $params;
		$newContent = json_encode($oldData);
		file_put_contents($filename, $newContent);
		echo json_encode(array('success' => true));
	} else {
		echo json_encode(array('success' => false, 'error' => array('code' => 1000, 'message' => 'File is not created yet')));
	}
} else {
	echo json_encode(array('success' => false, 'error' => array('code' => 1001, 'message' => 'Filename is empty')));
}
