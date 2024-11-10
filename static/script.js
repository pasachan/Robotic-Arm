// Function to send command to the server
function sendCommand(part, direction) {
  fetch(`/control?part=${part}&direction=${direction}`)
    .then(response => response.text())
    .then(data => {
      console.log(data);
      updateAngleDisplay(part, direction);
    });
}

// Function to update the displayed angle value
function updateAngleDisplay(part, direction) {
  const angleSpan = document.getElementById(part + 'Angle');
  let currentAngle = parseInt(angleSpan.textContent);

  if (direction === 'forward') {
    currentAngle = Math.min(currentAngle + 5, 180);
  } else if (direction === 'backward') {
    currentAngle = Math.max(currentAngle - 5, 0);
  }

  angleSpan.textContent = currentAngle + '°';
}

// Function to reset all angles to 0
function resetAll() {
  fetch('/reset')
    .then(response => response.text())
    .then(data => {
      console.log(data);
      document.getElementById('baseAngle').textContent = '0°';
      document.getElementById('link2Angle').textContent = '0°';
      document.getElementById('link1Angle').textContent = '0°';
      document.getElementById('gripperAngle').textContent = '0°';
    });
}

function performPick() {
  fetch('/performPick')
    .then(response => response.text())
    .then(data => {
      console.log(data);
      document.getElementById('baseAngle').textContent = '150°';
      document.getElementById('link2Angle').textContent = '160°';
      document.getElementById('link1Angle').textContent = '40°';
      document.getElementById('gripperAngle').textContent = '180°';
      setInterval(2000)
      document.getElementById('gripperAngle').textContent = '0°';
    });
}

function performPlace() {
  fetch('/performPlace')
    .then(response => response.text())
    .then(data => {
      console.log(data);
      document.getElementById('baseAngle').textContent = '20°';
      document.getElementById('link1Angle').textContent = '60°';
      document.getElementById('gripperAngle').textContent = '180°';
    });
}




