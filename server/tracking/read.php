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

$startTime = array_key_exists("startTime", $params) ? $params["startTime"] : null;
$endTime = array_key_exists("endTime", $params) ? $params["endTime"] : null;


if (!is_null($startTime)) {
	if (is_null(strtotime($startTime))) {
		echo json_encode(array('success' => false, 'error' => array('code' => 1002, 'message' => 'startTime format is not correct')));
		exit();
	} else {
		$startTime = strtotime($startTime);
	}
}
if (!is_null($endTime)) {
	if (is_null(strtotime($endTime))) {
		echo json_encode(array('success' => false, 'error' => array('code' => 1002, 'message' => 'endTime format is not correct')));
		exit();
	} else {
		$endTime = strtotime($endTime);
	}
}

if (!is_null($filename) && strcmp($filename, "") != 0) {
	$oldData = [];
	if (file_exists($filename)) {
		$content = file_get_contents($filename);
		$oldData = json_decode($content, true);

		$filteredData = array();
		foreach ($oldData as $value) {
			$correct = true;
			
			$createdAt = strtotime($value["createdAt"]);
			if (!is_null($startTime) && $createdAt < $startTime)
				$correct = false;
			if (!is_null($endTime) && $createdAt > $endTime)
				$correct = false;

			if ($correct)
				$filteredData[] = $value;
		}

		echo json_encode(array('success' => true, 'data' => $filteredData));
	} else {
		echo json_encode(array('success' => false, 'error' => array('code' => 1000, 'message' => 'File is not created yet')));
	}
} else {
	echo json_encode(array('success' => false, 'error' => array('code' => 1001, 'message' => 'Filename is empty')));
}
