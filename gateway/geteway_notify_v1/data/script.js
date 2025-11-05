function saveWificfg() {
  const ssid = document.getElementById("wifi_ssid").value.trim();
  const password = document.getElementById("wifi_password").value.trim();

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

function saveNotifycfg() {
  const radioButtons = document.querySelectorAll('input[name="notify"]');
  radioButtons.forEach((radio) => {
    if (this.checked) {
      switch (this.id) {
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
