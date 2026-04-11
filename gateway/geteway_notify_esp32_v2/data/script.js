window.addEventListener("load", async () => {
  await fetchWifiList();
  await getConfigWifi();
  await getConfigNotify();
});

let playload = {
  url:"",
  channel:"",
  type: 1,
  interval: 4,
  location: "โปรดระบุ"
};

function saveWificfg() {
  const ssid = (document.getElementById("wifi_ssid").value).trim();
  const password = (document.getElementById("wifi_password").value).trim();

  fetch("/save_wifi", {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({ ssid: ssid, password: password }),
  })
    .then((response) => {
      if (!response.ok) {
        throw new Error("Network response was not ok: " + response.statusText);
      }
      return response.json();
    })
    .then((data) => {
      alert("✅สำเร็จ!✅");
    })
    .catch((error) => {
      console.error("Error:", error);
      alert("⛔เกิดข้อผิดพลาด");
    });
}
// End 

function saveNotifycfg() {  
  playload.url = "";
  playload.channel = "";
  const radioButtons = document.querySelectorAll('input[name="notify"]');
  radioButtons.forEach((radio) => {    
    if (radio.checked) {
      // 1 = ntfy, 2 = telegram, 3 = discord
      switch (radio.id) {
        case "ntfy":
          playload.location = (document.getElementById("location").value).trim();
          playload.url = (document.getElementById("topic").value).trim();
          playload.type = parseInt(radio.value); 
          playload.interval = parseInt(document.getElementById("schedule").value);           
          sendConfigNotify(playload);
        break;
        case "telegram":
          playload.location = (document.getElementById("location").value).trim();
          playload.url = (document.getElementById("telegram_token").value).trim();
          playload.channel = (document.getElementById("telegram_channel").value).trim();
          playload.type = parseInt(radio.value); 
          playload.interval = parseInt(document.getElementById("schedule").value);           
          sendConfigNotify(playload);
        break;
        case "discord":
          playload.location = (document.getElementById("location").value).trim();
          playload.url = (document.getElementById("discord_webhook").value).trim();
          playload.type = parseInt(radio.value); 
          playload.interval = parseInt(document.getElementById("schedule").value);           
          sendConfigNotify(playload);
      }
    }
  });
}

function sendConfigNotify(playload) {  
  fetch("/save_notify", {
    method: "POST", 
    headers: {      
      "Content-Type": "application/json",
    },    
    body: JSON.stringify(playload),
  })
    .then((response) => {      
      if (!response.ok) {        
        throw new Error("Network response was not ok: " + response.statusText);
      }      
      return response.json();
    })
    .then((data) => {      
      console.log("Success:", data);
      alert("✅สำเร็จ!✅");    
    })
    .catch((error) => {      
      console.error("Error:", error);
      alert("⛔เกิดข้อผิดพลาด");    
    });
}

function showOptionNotify(data){
  switch (parseInt(data)) {
    case 1: //ntfy
      document.getElementsByClassName("ntfy")[0].style.display = "inline";
      document.getElementsByClassName("telegram")[0].style.display = "none";
      document.getElementsByClassName("discord")[0].style.display = "none";
      document.getElementById("btnSaveNotify").style.display = "inline";
      document.getElementById("notify-content").style.display = "block";
      break;
    case 2: //Telegram
      document.getElementsByClassName("ntfy")[0].style.display = "none";
      document.getElementsByClassName("telegram")[0].style.display = "inline";
      document.getElementsByClassName("discord")[0].style.display = "none";
      document.getElementById("btnSaveNotify").style.display = "inline";
      document.getElementById("notify-content").style.display = "block";
      break;
    case 3: //Discord
      document.getElementsByClassName("ntfy")[0].style.display = "none";
      document.getElementsByClassName("telegram")[0].style.display = "none";
      document.getElementsByClassName("discord")[0].style.display = "inline";
      document.getElementById("btnSaveNotify").style.display = "inline";
      document.getElementById("notify-content").style.display = "block";
      break;
  }
}

const radioButtons = document.querySelectorAll('input[name="notify"]');
radioButtons.forEach((radio) => {
  radio.addEventListener("change", function () {
    if (this.checked) {
      showOptionNotify(parseInt(this.value));
    }
  });
});


// ฟังก์ชันสำหรับโหลดหน้าเว็บใหม่
function Reload() {
  fetch("/reset");
  // หรือทำการเรียก API เพื่อให้อุปกรณ์รีบูต/รีโหลด
}

/**
* แสดง Modal นับถอยหลังก่อนเรียก Reload()
*/
function ReloadWithCountdown() {
  let count = 3;
  const modal = document.getElementById('countdownModal');
  const timerDisplay = document.getElementById('countdownTimer');

  // 1. แสดง Modal
  modal.style.display = 'block';
  timerDisplay.textContent = count;

  // ฟังก์ชันย่อยสำหรับนับถอยหลัง
  function countdown() {
      if (count > 0) {
          timerDisplay.textContent = count; // อัปเดตตัวเลขใน Modal
          count--;
          // หน่วงเวลา 1 วินาทีแล้วเรียกตัวเองซ้ำ
          setTimeout(countdown, 1000); // 1000ms = 1 วินาที
      } else {
          // เมื่อนับครบแล้ว
          modal.style.display = 'none'; // ซ่อน Modal
          Reload(); // เรียกฟังก์ชันโหลดหน้าเว็บ/รีบูต
      }
  }

  // เริ่มการนับถอยหลัง
  countdown();
}


let isScanning = false; // ตัวแปรป้องกันการสแกนซ้ำซ้อนขณะกำลังดึงข้อมูล

function fetchWifiList() {
  const dataList = document.getElementById('wifi_list'); 
  // ถ้ากำลังสแกนอยู่ ให้หยุดการทำงาน (ป้องกันการรัว API)
  if (isScanning) return;
  
  if(dataList){
    isScanning = true;
    
  }else{
    console.error("ไม่พบ Element ที่มี id='wifi_list'");
  }
  
  try {
    fetch('/getWifiList')
    .then(response => response.json())
    .then(data => {
      dataList.innerHTML = ''; // ล้างค่า "กำลังค้นหา" ออก
      
      if (data.length === 0) {
        dataList.innerHTML = '<option value="ไม่พบสัญญาณ WiFi">';
      } else {
        data.forEach(net => {
          // สร้างตัวเลือกจากรายการ WiFi ที่สแกนได้
          const option = document.createElement('option');
          option.value = net.ssid;
          // แสดงความแรงสัญญาณ (RSSI) กำกับไว้ข้างๆ เพื่อช่วยตัดสินใจ
          option.textContent = `${net.ssid} (${net.rssi} dBm)`; 
          dataList.appendChild(option);
        });
      }
      isScanning = false;
    })
    .catch(err => {
      console.error('Error fetching WiFi:', err);
      dataList.innerHTML = '<option value="เกิดข้อผิดพลาดในการโหลด">';
      isScanning = false;
    });
  } catch (error) {
    console.error("Fetch error:", error);
    dataList.innerHTML = '<option value="เกิดข้อผิดพลาดในการค้นหา">';
  } finally{
    isScanning = false;
  }
}

function isNotEmptyObject(obj) {
  return obj && Object.keys(obj).length > 0;
}

function getConfigNotify(){
  try {
    fetch('/getNotify')
    .then(response => response.json())
    .then(data => {     
      if(isNotEmptyObject(data)){
        const radioButtons = document.querySelectorAll('input[name="notify"]');
        radioButtons.forEach(ele => {                            
          if(ele.value == data.type){             
            ele.checked = true; 
          }
        });
        document.getElementById("location").value = data.location;  
        document.getElementById("schedule").value = data.interval;      
        switch(data.type){ // 1 = ntfy, 2 = telegram, 3 = discord
          case 1:
            document.getElementById("topic").value = data.url;
            showOptionNotify(data.type);
          break;
          case 2:
            document.getElementById("telegram_token").data.url;
            document.getElementById("telegram_channel").value = data.channel;
            showOptionNotify(data.type);
          break;
          case 3:
            document.getElementById("discord_webhook").data.url;   
            showOptionNotify(data.type);
          break;
        }
      }
    });
  } catch (error) {
    console.error('Error fetching WiFi:', error);
    dataList.innerHTML = '<option value="เกิดข้อผิดพลาดในการโหลด">';
  }
}
// 
function getConfigWifi(){
  try {
    fetch("/getWifi")
    .then(response => response.json())
    .then(data => {
      if(isNotEmptyObject(data)){
        document.getElementById("wifi_ssid").value = data.ssid;
        document.getElementById("wifi_password").value = data.password;
      }
    });
  } catch (error) {
    console.error('Error fetching WiFi:', error);
    dataList.innerHTML = '<option value="เกิดข้อผิดพลาดในการโหลด">';
  }
}
