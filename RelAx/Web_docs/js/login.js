var attempt = 3; // Variable to count number of attempts.
// Below function Executes on click of login button.

var nesne ;
if(navigator.appName.search('Microsoft')>-1) { nesne = new ActiveXObject('MSXML2.XMLHTTP'); }
else { nesne = new XMLHttpRequest(); }
nesne.timeout = 5000;

function validate()
{
	var username = document.getElementById("username").value;
	var password = document.getElementById("password").value;
	if ( username == "Admin" && password == "1234")
	{
		window.location = "dashboard.html"; // Redirecting to other page.
		return false;
	}
	else
	{
		attempt --;// Decrementing by one.
		document.getElementById("msg").innerHTML = "Логин или пароль неверны, осталось "+attempt+" попытки!"// Disabling fields after 3 attempts.
		if( attempt <= 0)
		{
			document.getElementById("msg").innerHTML = "Система заблокирована! Вызовите администратора!";
			window.location = "blocked.html";
			return false;
		}
	}
 }

function writeone() {
var username = document.getElementById("username").value;
var password = document.getElementById("password").value;

nesne.onreadystatechange = function() {
if (this.readyState == 4 && this.status == 200) {
		window.location = "";
  }
};

nesne.open("post", "", true);
nesne.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
nesne.send("validate\n" + username + " " + password);
attempt --;// Decrementing by one.
document.getElementById("msg").innerHTML = "Логин или пароль неверны, осталось "+attempt+" попытки!";// Disabling fields after 3 attempts.
if( attempt <= 0)
{
	document.getElementById("msg").innerHTML = "Система заблокирована! Вызовите администратора!";
	window.location = "blocked.html";
	return false;
}
}

document.onkeydown = function(e) {
    if (e.keyCode == 13) {
        writeone();
    }
}

function logout() {
nesne.onreadystatechange = function() {
if (this.readyState == 4 && this.status == 200) {
		window.location = "";
  }
};
nesne.open("post", "", true);
nesne.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
nesne.send("logout\n");
}

