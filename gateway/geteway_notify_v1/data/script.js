const radioButtons = document.querySelectorAll('input[name="notify"]');
      // วนลูปเพื่อเพิ่ม Event Listener ให้แต่ละตัว
      radioButtons.forEach((radio) => {
        radio.addEventListener("change", function () {
          if (this.checked) {
            console.log("ตัวเลือกที่ถูกเลือกคือ:", this.id);
            console.log("Value คือ:", this.value);
            switch (this.id) {
              case "ntfy":
                document.getElementsByClassName("ntfy")[0].style.display =
                  "inline";
                document.getElementsByClassName("telegram")[0].style.display =
                  "none";
                document.getElementsByClassName("discord")[0].style.display =
                  "none";
                document.getElementById("btnSaveNotify").style.display =
                  "inline";
                document.getElementById("notify-content").style.display =
                  "block";
                break;
              case "telegram":
                document.getElementsByClassName("ntfy")[0].style.display =
                  "none";
                document.getElementsByClassName("telegram")[0].style.display =
                  "inline";
                document.getElementsByClassName("discord")[0].style.display =
                  "none";
                document.getElementById("btnSaveNotify").style.display =
                  "inline";
                document.getElementById("notify-content").style.display =
                  "block";
                break;
              case "discord":
                document.getElementsByClassName("ntfy")[0].style.display =
                  "none";
                document.getElementsByClassName("telegram")[0].style.display =
                  "none";
                document.getElementsByClassName("discord")[0].style.display =
                  "inline";
                document.getElementById("btnSaveNotify").style.display =
                  "inline";
                document.getElementById("notify-content").style.display =
                  "block";
                break;
            }
          }
        });
      });