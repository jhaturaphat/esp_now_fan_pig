let playload = {
  url:"",
  channel:"",
  type: 1,
  interval: 4
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
          playload.url = (document.getElementById("topic").value).trim();
          playload.type = parseInt(radio.value); 
          playload.interval = parseInt(document.getElementById("schedule").value);           
          sendConfigNotify(playload);
        break;
        case "telegram":
          playload.url = (document.getElementById("telegram_token").value).trim();
          playload.channel = (document.getElementById("telegram_channel").value).trim();
          playload.type = parseInt(radio.value); 
          playload.interval = parseInt(document.getElementById("schedule").value);           
          sendConfigNotify(playload);
        break;
        case "discord":
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

const radioButtons = document.querySelectorAll('input[name="notify"]');
radioButtons.forEach((radio) => {
  radio.addEventListener("change", function () {
    if (this.checked) {
      switch (this.id) {
        case "ntfy":
          document.getElementsByClassName("ntfy")[0].style.display = "inline";
          document.getElementsByClassName("telegram")[0].style.display = "none";
          document.getElementsByClassName("discord")[0].style.display = "none";
          document.getElementById("btnSaveNotify").style.display = "inline";
          document.getElementById("notify-content").style.display = "block";
          break;
        case "telegram":
          document.getElementsByClassName("ntfy")[0].style.display = "none";
          document.getElementsByClassName("telegram")[0].style.display =
            "inline";
          document.getElementsByClassName("discord")[0].style.display = "none";
          document.getElementById("btnSaveNotify").style.display = "inline";
          document.getElementById("notify-content").style.display = "block";
          break;
        case "discord":
          document.getElementsByClassName("ntfy")[0].style.display = "none";
          document.getElementsByClassName("telegram")[0].style.display = "none";
          document.getElementsByClassName("discord")[0].style.display =
            "inline";
          document.getElementById("btnSaveNotify").style.display = "inline";
          document.getElementById("notify-content").style.display = "block";
          break;
      }
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
