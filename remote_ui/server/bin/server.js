var express = require('express');
var app = express();

app.use(express.static('../web'));	

app.listen(9000);
console.log('Server listening on http://localhost:9000');