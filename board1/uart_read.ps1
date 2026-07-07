param(
    [string]$Port = "COM5",
    [int]$Baud = 115200
)

$labels = @(
    "a_little",
    "good",
    "help",
    "me",
    "name",
    "sign_language",
    "you"
)

function Get-Crc8 {
    param([byte[]]$Data)

    [int]$crc = 0
    foreach ($b in $Data) {
        $crc = $crc -bxor [int]$b
        for ($i = 0; $i -lt 8; $i++) {
            if (($crc -band 0x80) -ne 0) {
                $crc = (($crc -shl 1) -bxor 0x07) -band 0xff
            } else {
                $crc = ($crc -shl 1) -band 0xff
            }
        }
    }

    return [byte]$crc
}

function Read-Exact {
    param(
        [System.IO.Ports.SerialPort]$SerialPort,
        [int]$Length
    )

    $buffer = New-Object byte[] $Length
    $offset = 0

    while ($offset -lt $Length) {
        $read = $SerialPort.Read($buffer, $offset, $Length - $offset)
        if ($read -gt 0) {
            $offset += $read
        }
    }

    return $buffer
}

$serial = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$serial.ReadTimeout = 1000

try {
    $serial.Open()
    Write-Host "Listening on $Port @ $Baud 8N1"
    Write-Host "Frame: AA 55 01 01 seq gesture_id confidence status crc8"
    Write-Host "Press Ctrl+C to stop."

    while ($true) {
        try {
            $b0 = $serial.ReadByte()
        } catch [System.TimeoutException] {
            continue
        }

        if ($b0 -ne 0xAA) {
            continue
        }

        try {
            $b1 = $serial.ReadByte()
        } catch [System.TimeoutException] {
            continue
        }

        if ($b1 -ne 0x55) {
            continue
        }

        try {
            $rest = Read-Exact -SerialPort $serial -Length 7
        } catch [System.TimeoutException] {
            Write-Host "Timeout while reading frame body"
            continue
        }

        $frame = New-Object byte[] 9
        $frame[0] = 0xAA
        $frame[1] = 0x55
        [Array]::Copy($rest, 0, $frame, 2, 7)

        $crcData = New-Object byte[] 6
        [Array]::Copy($frame, 2, $crcData, 0, 6)
        $calcCrc = Get-Crc8 -Data $crcData
        $recvCrc = $frame[8]

        $hex = ($frame | ForEach-Object { $_.ToString("X2") }) -join " "

        if ($calcCrc -ne $recvCrc) {
            Write-Host "CRC error: $hex calc=$($calcCrc.ToString("X2")) recv=$($recvCrc.ToString("X2"))"
            continue
        }

        $version = $frame[2]
        $frameType = $frame[3]
        $seq = $frame[4]
        $gestureId = $frame[5]
        $confidence = $frame[6]
        $status = $frame[7]

        if ($version -ne 0x01 -or $frameType -ne 0x01) {
            Write-Host "Unknown frame: $hex"
            continue
        }

        if ($gestureId -lt $labels.Count) {
            $label = $labels[$gestureId]
        } else {
            $label = "unknown"
        }

        $valid = (($status -band 0x01) -ne 0)
        $lowConfidence = (($status -band 0x02) -ne 0)

        Write-Host ("seq={0,3} id={1} label={2,-14} conf={3,3}% valid={4,-5} low_conf={5,-5} raw={6}" -f `
            $seq, $gestureId, $label, $confidence, $valid, $lowConfidence, $hex)
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
