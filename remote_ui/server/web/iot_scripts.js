document.onkeydown = checkKey;
document.onkeyup = releaseKey;

var addr = "http://192.168.4.1/console/send?text=";
// JSONs defined as Strings because JSON.stringify(jsonObject)
// does quotes("") for mdata and it's not working
var speed = 150;
var turnSpeed = 90;
var forward = "{\"command\":\"drive\",mdata:[1,"+speed+",50000]}";
var backward = "{\"command\":\"drive\",mdata:[2,"+speed+",50000]}";
var left = "{\"command\":\"drive\",mdata:[4,"+turnSpeed+",50000]}";
var right = "{\"command\":\"drive\",mdata:[3,"+turnSpeed+",50000]}";
var stop =  "{\"command\":\"drive\",mdata:[,,]}";
var automation =  "{\"command\":\"drive\",mdata:[5,,]}";

var UP_KEY = '38';
var DOWN_KEY = '40';
var LEFT_KEY = '37';
var RIGHT_KEY = '39';
var SPACE_KEY = '32';
var ENTER_KEY = '13';

var RELEASED_KEY = -1;
var keyDown = RELEASED_KEY;

function releaseKey(e) {
	if (!isControllerKey(e)) {
		return;
	}
	console.log("Key released");
	executeCommand(stop);
	keyDown = RELEASED_KEY;
}

function checkKey(e) {

    e = e || window.event;
	
	if (keyDown == RELEASED_KEY) {
		keyDown = e.keyCode;
	} else {
		return;
	}
	
    if (e.keyCode == UP_KEY) {
		executeCommand(forward);
        console.log("up");
    }
    else if (e.keyCode == DOWN_KEY) {
		executeCommand(backward);
        console.log("down");
    }
    else if (e.keyCode == LEFT_KEY) {
	   executeCommand(left);
       console.log("left");
    }
    else if (e.keyCode == RIGHT_KEY) {
	   executeCommand(right);
       console.log("right");
    } 
	else if (e.keyCode == SPACE_KEY) {
	   executeCommand(stop);
       console.log("stop");
    } else if (ENTER_KEY) {
		executeCommand(automation);
		console.log("automation on...");
	}
}

function isControllerKey(e) {
	return e.keyCode == UP_KEY ||
			e.keyCode == DOWN_KEY ||
			e.keyCode == LEFT_KEY ||
			e.keyCode == RIGHT_KEY ||
			e.keyCode == SPACE_KEY;
}

function executeCommand(cmd) {
	$.ajax({
            url: addr + encodeURIComponent(cmd + "\r\n"),
            dataType: 'jsonp',
			crossDomain: true,
			headers: {
				'Access-Control-Allow-Origin': '*'
			},
            always: function (data) {
                console.log(data);
            }
        });
}