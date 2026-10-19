// หน้าเว็บ HTML สำหรับให้มือถือเข้ามาตั้งค่า
const char config_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP-01 Wi-Fi Setup</title>
  <!-- บรรทัดสำคัญ: กำหนดให้ขนาดเว็บพอดีกับหน้าจออุปกรณ์เคลื่อนที่ -->
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <meta charset="UTF-8">
  <style>
    /* ตั้งค่ากล่องข้อความและองค์ประกอบทั้งหมดให้คำนวณขนาดรวม Padding */
    * { box-sizing: border-box; margin: 0; padding: 0; }
    
    html, body { 
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; 
      background-color: #f0f2f5; 
      display: flex; 
      justify-content: center; 
      align-items: center; 
      min-height: 100vh;
      width: 100%;
    }
    
    /* กล่อง Card ปรับขนาดตามหน้าจออัตโนมัติ */
    .card { 
      background: #ffffff; 
      padding: 32px 24px; 
      border-radius: 16px; 
      box-shadow: 0 8px 24px rgba(0,0,0,0.08); 
      width: 90%;          /* กว้าง 90% ของหน้าจอมือถือ */
      max-width: 360px;    /* แต่กว้างสุดไม่เกิน 360px บนจอใหญ่ */
      text-align: center; 
    }
    
    h2 { 
      color: #1a1a1a; 
      font-size: 24px;
      margin-bottom: 8px; 
    }

    p {
      color: #666666;
      font-size: 14px;
      margin-bottom: 24px;
    }
    
    /* กล่องข้อความ Input ขนาดใหญ่ กดง่ายบนมือถือ */
    input[type="text"], input[type="password"] { 
      width: 100%; 
      padding: 14px 16px; 
      margin-bottom: 16px; 
      border: 1px solid #e0e0e0; 
      border-radius: 8px; 
      font-size: 16px;     /* ขนาด 16px ป้องกันไม่ให้ iOS ซูมหน้าจออัตโนมัติเวลาพิมพ์ */
      background-color: #f8f9fa;
      transition: border-color 0.2s, background-color 0.2s;
    }

    input[type="text"]:focus, input[type="password"]:focus {
      border-color: #007aff;
      background-color: #ffffff;
      outline: none;
    }
    
    /* ปุ่มกดขนาดใหญ่ เต็มความกว้าง */
    input[type="submit"] { 
      width: 100%; 
      background-color: #007aff; 
      color: white; 
      padding: 14px; 
      margin-top: 8px; 
      border: none; 
      border-radius: 8px; 
      font-size: 16px; 
      cursor: pointer; 
      font-weight: 600; 
      box-shadow: 0 4px 12px rgba(0, 122, 255, 0.2);
      -webkit-tap-highlight-color: transparent;
    }

    input[type="submit"]:active { 
      background-color: #0056b3; 
    }
  </style>
</head>
<body>
  <div class="card">
    <h2>Wi-Fi Setup</h2>
    <p>กรุณากรอกข้อมูลเพื่อเชื่อมต่ออินเทอร์เน็ต</p>
    <form action="/save" method="POST">
      <input type="text" name="ssid" value="%SSID%" placeholder="ชื่อ Wi-Fi (SSID)" required autocomplete="off">
      <input type="text" name="password" value="%PASS%" placeholder="รหัสผ่าน (Password)">
      <input type="submit" value="บันทึกและเชื่อมต่อ">
    </form>
  </div>
</body>
</html>
)rawliteral";
