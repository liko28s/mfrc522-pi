var MFRC522 = function(){};
MFRC522.prototype.spawn = require('child_process').spawn;
MFRC522.prototype.mfrc522py = null;
MFRC522.prototype.callback = null;
MFRC522.prototype.dataRegex = new RegExp('"UID": [[0-9,]{4}');

MFRC522.prototype.start = function (callback) {
    this.callback = callback;

    this.mfrc522py = this.spawn('python3', ['-u', __dirname+'/dump.py']);

    var _this = this;
    this.mfrc522py.stdout.on('data', function (data) {
        var card = data.toString('utf8');
        end_json = _this.dataRegex.exec(card);
        if(end_json =! null) {
            _this.callback.onData(card)
        }
    });

    this.mfrc522py.on('exit', function () {
        _this.callback.onExit();
    });

    _this.callback.onStart();
};

mfrc522 = new MFRC522();

var Callback = function () {
    this.onStart = function () {
        console.log('Waiting Cards');
    };

    this.onData = function (data) {
        var dataJson = JSON.parse(data);
        console.log('Read Card');
    };

    this.onExit = function () {
        console.log('Exit');
    }
};

mfrc522.start(new Callback());