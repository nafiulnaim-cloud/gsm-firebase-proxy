<?php
$data = file_get_contents("php://input");

if (!$data) {
  echo json_encode(["error" => "No data supplied."]);
  exit;
}

$firebaseUrl = "https://gsm-tracker-student-default-rtdb.asia-southeast1.firebasedatabase.app/data.json?auth=f83fnNYQVJog9dwUUZrSGbtUM9k3DhYopTDtZFFyT";

$ch = curl_init($firebaseUrl);
curl_setopt($ch, CURLOPT_CUSTOMREQUEST, "POST");
curl_setopt($ch, CURLOPT_POSTFIELDS, $data);
curl_setopt($ch, CURLOPT_RETURNTRANSFER, true);
curl_setopt($ch, CURLOPT_HTTPHEADER, ['Content-Type: application/json']);
$response = curl_exec($ch);

if (curl_errno($ch)) {
  echo json_encode(["error" => curl_error($ch)]);
} else {
  echo $response;
}

curl_close($ch);
?>
