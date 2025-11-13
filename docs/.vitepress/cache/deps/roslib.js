import {
  __commonJS
} from "./chunk-PR4QN5HX.js";

// node_modules/object-assign/index.js
var require_object_assign = __commonJS({
  "node_modules/object-assign/index.js"(exports, module) {
    "use strict";
    var getOwnPropertySymbols = Object.getOwnPropertySymbols;
    var hasOwnProperty = Object.prototype.hasOwnProperty;
    var propIsEnumerable = Object.prototype.propertyIsEnumerable;
    function toObject(val) {
      if (val === null || val === void 0) {
        throw new TypeError("Object.assign cannot be called with null or undefined");
      }
      return Object(val);
    }
    function shouldUseNative() {
      try {
        if (!Object.assign) {
          return false;
        }
        var test1 = new String("abc");
        test1[5] = "de";
        if (Object.getOwnPropertyNames(test1)[0] === "5") {
          return false;
        }
        var test2 = {};
        for (var i = 0; i < 10; i++) {
          test2["_" + String.fromCharCode(i)] = i;
        }
        var order2 = Object.getOwnPropertyNames(test2).map(function(n) {
          return test2[n];
        });
        if (order2.join("") !== "0123456789") {
          return false;
        }
        var test3 = {};
        "abcdefghijklmnopqrst".split("").forEach(function(letter) {
          test3[letter] = letter;
        });
        if (Object.keys(Object.assign({}, test3)).join("") !== "abcdefghijklmnopqrst") {
          return false;
        }
        return true;
      } catch (err) {
        return false;
      }
    }
    module.exports = shouldUseNative() ? Object.assign : function(target, source) {
      var from;
      var to = toObject(target);
      var symbols;
      for (var s = 1; s < arguments.length; s++) {
        from = Object(arguments[s]);
        for (var key in from) {
          if (hasOwnProperty.call(from, key)) {
            to[key] = from[key];
          }
        }
        if (getOwnPropertySymbols) {
          symbols = getOwnPropertySymbols(from);
          for (var i = 0; i < symbols.length; i++) {
            if (propIsEnumerable.call(from, symbols[i])) {
              to[symbols[i]] = from[symbols[i]];
            }
          }
        }
      }
      return to;
    };
  }
});

// node_modules/roslib/src/mixin.js
var require_mixin = __commonJS({
  "node_modules/roslib/src/mixin.js"(exports, module) {
    module.exports = function(Ros, classes, features) {
      classes.forEach(function(className) {
        var Class = features[className];
        Ros.prototype[className] = function(options) {
          options.ros = this;
          return new Class(options);
        };
      });
    };
  }
});

// node_modules/roslib/src/util/shim/WebSocket.js
var require_WebSocket = __commonJS({
  "node_modules/roslib/src/util/shim/WebSocket.js"(exports, module) {
    module.exports = typeof window !== "undefined" ? window.WebSocket : WebSocket;
  }
});

// node_modules/webworkify/index.js
var require_webworkify = __commonJS({
  "node_modules/webworkify/index.js"(exports, module) {
    var bundleFn = arguments[3];
    var sources = arguments[4];
    var cache = arguments[5];
    var stringify = JSON.stringify;
    module.exports = function(fn, options) {
      var wkey;
      var cacheKeys = Object.keys(cache);
      for (var i = 0, l = cacheKeys.length; i < l; i++) {
        var key = cacheKeys[i];
        var exp = cache[key].exports;
        if (exp === fn || exp && exp.default === fn) {
          wkey = key;
          break;
        }
      }
      if (!wkey) {
        wkey = Math.floor(Math.pow(16, 8) * Math.random()).toString(16);
        var wcache = {};
        for (var i = 0, l = cacheKeys.length; i < l; i++) {
          var key = cacheKeys[i];
          wcache[key] = key;
        }
        sources[wkey] = [
          "function(require,module,exports){" + fn + "(self); }",
          wcache
        ];
      }
      var skey = Math.floor(Math.pow(16, 8) * Math.random()).toString(16);
      var scache = {};
      scache[wkey] = wkey;
      sources[skey] = [
        "function(require,module,exports){var f = require(" + stringify(wkey) + ");(f.default ? f.default : f)(self);}",
        scache
      ];
      var workerSources = {};
      resolveSources(skey);
      function resolveSources(key2) {
        workerSources[key2] = true;
        for (var depPath in sources[key2][1]) {
          var depKey = sources[key2][1][depPath];
          if (!workerSources[depKey]) {
            resolveSources(depKey);
          }
        }
      }
      var src = "(" + bundleFn + ")({" + Object.keys(workerSources).map(function(key2) {
        return stringify(key2) + ":[" + sources[key2][0] + "," + stringify(sources[key2][1]) + "]";
      }).join(",") + "},{},[" + stringify(skey) + "])";
      var URL = window.URL || window.webkitURL || window.mozURL || window.msURL;
      var blob = new Blob([src], { type: "text/javascript" });
      if (options && options.bare) {
        return blob;
      }
      var workerUrl = URL.createObjectURL(blob);
      var worker = new Worker(workerUrl);
      worker.objectURL = workerUrl;
      return worker;
    };
  }
});

// node_modules/webworkify-webpack/index.js
var require_webworkify_webpack = __commonJS({
  "node_modules/webworkify-webpack/index.js"(exports, module) {
    function webpackBootstrapFunc(modules) {
      var installedModules = {};
      function __webpack_require__2(moduleId) {
        if (installedModules[moduleId])
          return installedModules[moduleId].exports;
        var module2 = installedModules[moduleId] = {
          /******/
          i: moduleId,
          /******/
          l: false,
          /******/
          exports: {}
          /******/
        };
        modules[moduleId].call(module2.exports, module2, module2.exports, __webpack_require__2);
        module2.l = true;
        return module2.exports;
      }
      __webpack_require__2.m = modules;
      __webpack_require__2.c = installedModules;
      __webpack_require__2.i = function(value) {
        return value;
      };
      __webpack_require__2.d = function(exports2, name, getter) {
        if (!__webpack_require__2.o(exports2, name)) {
          Object.defineProperty(exports2, name, {
            /******/
            configurable: false,
            /******/
            enumerable: true,
            /******/
            get: getter
            /******/
          });
        }
      };
      __webpack_require__2.r = function(exports2) {
        Object.defineProperty(exports2, "__esModule", { value: true });
      };
      __webpack_require__2.n = function(module2) {
        var getter = module2 && module2.__esModule ? (
          /******/
          function getDefault() {
            return module2["default"];
          }
        ) : (
          /******/
          function getModuleExports() {
            return module2;
          }
        );
        __webpack_require__2.d(getter, "a", getter);
        return getter;
      };
      __webpack_require__2.o = function(object, property) {
        return Object.prototype.hasOwnProperty.call(object, property);
      };
      __webpack_require__2.p = "/";
      __webpack_require__2.oe = function(err) {
        console.error(err);
        throw err;
      };
      var f = __webpack_require__2(__webpack_require__2.s = ENTRY_MODULE);
      return f.default || f;
    }
    var moduleNameReqExp = "[\\.|\\-|\\+|\\w|/|@]+";
    var dependencyRegExp = "\\(\\s*(/\\*.*?\\*/)?\\s*.*?(" + moduleNameReqExp + ").*?\\)";
    function quoteRegExp(str) {
      return (str + "").replace(/[.?*+^$[\]\\(){}|-]/g, "\\$&");
    }
    function isNumeric(n) {
      return !isNaN(1 * n);
    }
    function getModuleDependencies(sources, module2, queueName) {
      var retval = {};
      retval[queueName] = [];
      var fnString = module2.toString();
      var wrapperSignature = fnString.match(/^function\s?\w*\(\w+,\s*\w+,\s*(\w+)\)/);
      if (!wrapperSignature) return retval;
      var webpackRequireName = wrapperSignature[1];
      var re = new RegExp("(\\\\n|\\W)" + quoteRegExp(webpackRequireName) + dependencyRegExp, "g");
      var match;
      while (match = re.exec(fnString)) {
        if (match[3] === "dll-reference") continue;
        retval[queueName].push(match[3]);
      }
      re = new RegExp("\\(" + quoteRegExp(webpackRequireName) + '\\("(dll-reference\\s(' + moduleNameReqExp + '))"\\)\\)' + dependencyRegExp, "g");
      while (match = re.exec(fnString)) {
        if (!sources[match[2]]) {
          retval[queueName].push(match[1]);
          sources[match[2]] = __webpack_require__(match[1]).m;
        }
        retval[match[2]] = retval[match[2]] || [];
        retval[match[2]].push(match[4]);
      }
      var keys = Object.keys(retval);
      for (var i = 0; i < keys.length; i++) {
        for (var j = 0; j < retval[keys[i]].length; j++) {
          if (isNumeric(retval[keys[i]][j])) {
            retval[keys[i]][j] = 1 * retval[keys[i]][j];
          }
        }
      }
      return retval;
    }
    function hasValuesInQueues(queues) {
      var keys = Object.keys(queues);
      return keys.reduce(function(hasValues, key) {
        return hasValues || queues[key].length > 0;
      }, false);
    }
    function getRequiredModules(sources, moduleId) {
      var modulesQueue = {
        main: [moduleId]
      };
      var requiredModules = {
        main: []
      };
      var seenModules = {
        main: {}
      };
      while (hasValuesInQueues(modulesQueue)) {
        var queues = Object.keys(modulesQueue);
        for (var i = 0; i < queues.length; i++) {
          var queueName = queues[i];
          var queue = modulesQueue[queueName];
          var moduleToCheck = queue.pop();
          seenModules[queueName] = seenModules[queueName] || {};
          if (seenModules[queueName][moduleToCheck] || !sources[queueName][moduleToCheck]) continue;
          seenModules[queueName][moduleToCheck] = true;
          requiredModules[queueName] = requiredModules[queueName] || [];
          requiredModules[queueName].push(moduleToCheck);
          var newModules = getModuleDependencies(sources, sources[queueName][moduleToCheck], queueName);
          var newModulesKeys = Object.keys(newModules);
          for (var j = 0; j < newModulesKeys.length; j++) {
            modulesQueue[newModulesKeys[j]] = modulesQueue[newModulesKeys[j]] || [];
            modulesQueue[newModulesKeys[j]] = modulesQueue[newModulesKeys[j]].concat(newModules[newModulesKeys[j]]);
          }
        }
      }
      return requiredModules;
    }
    module.exports = function(moduleId, options) {
      options = options || {};
      var sources = {
        main: __webpack_modules__
      };
      var requiredModules = options.all ? { main: Object.keys(sources.main) } : getRequiredModules(sources, moduleId);
      var src = "";
      Object.keys(requiredModules).filter(function(m) {
        return m !== "main";
      }).forEach(function(module2) {
        var entryModule = 0;
        while (requiredModules[module2][entryModule]) {
          entryModule++;
        }
        requiredModules[module2].push(entryModule);
        sources[module2][entryModule] = "(function(module, exports, __webpack_require__) { module.exports = __webpack_require__; })";
        src = src + "var " + module2 + " = (" + webpackBootstrapFunc.toString().replace("ENTRY_MODULE", JSON.stringify(entryModule)) + ")({" + requiredModules[module2].map(function(id) {
          return "" + JSON.stringify(id) + ": " + sources[module2][id].toString();
        }).join(",") + "});\n";
      });
      src = src + "new ((" + webpackBootstrapFunc.toString().replace("ENTRY_MODULE", JSON.stringify(moduleId)) + ")({" + requiredModules.main.map(function(id) {
        return "" + JSON.stringify(id) + ": " + sources.main[id].toString();
      }).join(",") + "}))(self);";
      var blob = new window.Blob([src], { type: "text/javascript" });
      if (options.bare) {
        return blob;
      }
      var URL = window.URL || window.webkitURL || window.mozURL || window.msURL;
      var workerUrl = URL.createObjectURL(blob);
      var worker = new window.Worker(workerUrl);
      worker.objectURL = workerUrl;
      return worker;
    };
  }
});

// node_modules/roslib/src/util/workerSocketImpl.js
var require_workerSocketImpl = __commonJS({
  "node_modules/roslib/src/util/workerSocketImpl.js"(exports, module) {
    var WebSocket2 = WebSocket2 || require_WebSocket();
    module.exports = function(self) {
      var socket = null;
      function handleSocketMessage(ev) {
        var data = ev.data;
        if (data instanceof ArrayBuffer) {
          self.postMessage(data, [data]);
        } else {
          self.postMessage(data);
        }
      }
      function handleSocketControl(ev) {
        self.postMessage({ type: ev.type });
      }
      self.addEventListener("message", function(ev) {
        var data = ev.data;
        if (typeof data === "string") {
          socket.send(data);
        } else {
          if (data.hasOwnProperty("close")) {
            socket.close();
            socket = null;
          } else if (data.hasOwnProperty("uri")) {
            var uri = data.uri;
            socket = new WebSocket2(uri);
            socket.binaryType = "arraybuffer";
            socket.onmessage = handleSocketMessage;
            socket.onclose = handleSocketControl;
            socket.onopen = handleSocketControl;
            socket.onerror = handleSocketControl;
          } else {
            throw "Unknown message to WorkerSocket";
          }
        }
      });
    };
  }
});

// node_modules/roslib/src/util/workerSocket.js
var require_workerSocket = __commonJS({
  "node_modules/roslib/src/util/workerSocket.js"(exports, module) {
    try {
      work = require_webworkify();
    } catch (ReferenceError) {
      work = require_webworkify_webpack();
    }
    var work;
    var workerSocketImpl = require_workerSocketImpl();
    function WorkerSocket(uri) {
      this.socket_ = work(workerSocketImpl);
      this.socket_.addEventListener("message", this.handleWorkerMessage_.bind(this));
      this.socket_.postMessage({
        uri
      });
    }
    WorkerSocket.prototype.handleWorkerMessage_ = function(ev) {
      var data = ev.data;
      if (data instanceof ArrayBuffer || typeof data === "string") {
        this.onmessage(ev);
      } else {
        var type = data.type;
        if (type === "close") {
          this.onclose(null);
        } else if (type === "open") {
          this.onopen(null);
        } else if (type === "error") {
          this.onerror(null);
        } else {
          throw "Unknown message from workersocket";
        }
      }
    };
    WorkerSocket.prototype.send = function(data) {
      this.socket_.postMessage(data);
    };
    WorkerSocket.prototype.close = function() {
      this.socket_.postMessage({
        close: true
      });
    };
    module.exports = WorkerSocket;
  }
});

// node_modules/roslib/src/util/shim/canvas.js
var require_canvas = __commonJS({
  "node_modules/roslib/src/util/shim/canvas.js"(exports, module) {
    module.exports = function Canvas() {
      return document.createElement("canvas");
    };
  }
});

// node_modules/roslib/src/util/shim/decompressPng.js
var require_decompressPng = __commonJS({
  "node_modules/roslib/src/util/shim/decompressPng.js"(exports, module) {
    "use strict";
    var Canvas = require_canvas();
    var Image = Canvas.Image || window.Image;
    function decompressPng(data, callback) {
      var image = new Image();
      image.onload = function() {
        var canvas = new Canvas();
        var context = canvas.getContext("2d");
        canvas.width = image.width;
        canvas.height = image.height;
        context.imageSmoothingEnabled = false;
        context.webkitImageSmoothingEnabled = false;
        context.mozImageSmoothingEnabled = false;
        context.drawImage(image, 0, 0);
        var imageData = context.getImageData(0, 0, image.width, image.height).data;
        var jsonData = "";
        for (var i = 0; i < imageData.length; i += 4) {
          jsonData += String.fromCharCode(imageData[i], imageData[i + 1], imageData[i + 2]);
        }
        callback(JSON.parse(jsonData));
      };
      image.src = "data:image/png;base64," + data;
    }
    module.exports = decompressPng;
  }
});

// node_modules/cbor-js/cbor.js
var require_cbor = __commonJS({
  "node_modules/cbor-js/cbor.js"(exports, module) {
    (function(global, undefined2) {
      "use strict";
      var POW_2_24 = Math.pow(2, -24), POW_2_32 = Math.pow(2, 32), POW_2_53 = Math.pow(2, 53);
      function encode(value) {
        var data = new ArrayBuffer(256);
        var dataView = new DataView(data);
        var lastLength;
        var offset = 0;
        function ensureSpace(length) {
          var newByteLength = data.byteLength;
          var requiredLength = offset + length;
          while (newByteLength < requiredLength)
            newByteLength *= 2;
          if (newByteLength !== data.byteLength) {
            var oldDataView = dataView;
            data = new ArrayBuffer(newByteLength);
            dataView = new DataView(data);
            var uint32count = offset + 3 >> 2;
            for (var i2 = 0; i2 < uint32count; ++i2)
              dataView.setUint32(i2 * 4, oldDataView.getUint32(i2 * 4));
          }
          lastLength = length;
          return dataView;
        }
        function write() {
          offset += lastLength;
        }
        function writeFloat64(value2) {
          write(ensureSpace(8).setFloat64(offset, value2));
        }
        function writeUint8(value2) {
          write(ensureSpace(1).setUint8(offset, value2));
        }
        function writeUint8Array(value2) {
          var dataView2 = ensureSpace(value2.length);
          for (var i2 = 0; i2 < value2.length; ++i2)
            dataView2.setUint8(offset + i2, value2[i2]);
          write();
        }
        function writeUint16(value2) {
          write(ensureSpace(2).setUint16(offset, value2));
        }
        function writeUint32(value2) {
          write(ensureSpace(4).setUint32(offset, value2));
        }
        function writeUint64(value2) {
          var low = value2 % POW_2_32;
          var high = (value2 - low) / POW_2_32;
          var dataView2 = ensureSpace(8);
          dataView2.setUint32(offset, high);
          dataView2.setUint32(offset + 4, low);
          write();
        }
        function writeTypeAndLength(type, length) {
          if (length < 24) {
            writeUint8(type << 5 | length);
          } else if (length < 256) {
            writeUint8(type << 5 | 24);
            writeUint8(length);
          } else if (length < 65536) {
            writeUint8(type << 5 | 25);
            writeUint16(length);
          } else if (length < 4294967296) {
            writeUint8(type << 5 | 26);
            writeUint32(length);
          } else {
            writeUint8(type << 5 | 27);
            writeUint64(length);
          }
        }
        function encodeItem(value2) {
          var i2;
          if (value2 === false)
            return writeUint8(244);
          if (value2 === true)
            return writeUint8(245);
          if (value2 === null)
            return writeUint8(246);
          if (value2 === undefined2)
            return writeUint8(247);
          switch (typeof value2) {
            case "number":
              if (Math.floor(value2) === value2) {
                if (0 <= value2 && value2 <= POW_2_53)
                  return writeTypeAndLength(0, value2);
                if (-POW_2_53 <= value2 && value2 < 0)
                  return writeTypeAndLength(1, -(value2 + 1));
              }
              writeUint8(251);
              return writeFloat64(value2);
            case "string":
              var utf8data = [];
              for (i2 = 0; i2 < value2.length; ++i2) {
                var charCode = value2.charCodeAt(i2);
                if (charCode < 128) {
                  utf8data.push(charCode);
                } else if (charCode < 2048) {
                  utf8data.push(192 | charCode >> 6);
                  utf8data.push(128 | charCode & 63);
                } else if (charCode < 55296) {
                  utf8data.push(224 | charCode >> 12);
                  utf8data.push(128 | charCode >> 6 & 63);
                  utf8data.push(128 | charCode & 63);
                } else {
                  charCode = (charCode & 1023) << 10;
                  charCode |= value2.charCodeAt(++i2) & 1023;
                  charCode += 65536;
                  utf8data.push(240 | charCode >> 18);
                  utf8data.push(128 | charCode >> 12 & 63);
                  utf8data.push(128 | charCode >> 6 & 63);
                  utf8data.push(128 | charCode & 63);
                }
              }
              writeTypeAndLength(3, utf8data.length);
              return writeUint8Array(utf8data);
            default:
              var length;
              if (Array.isArray(value2)) {
                length = value2.length;
                writeTypeAndLength(4, length);
                for (i2 = 0; i2 < length; ++i2)
                  encodeItem(value2[i2]);
              } else if (value2 instanceof Uint8Array) {
                writeTypeAndLength(2, value2.length);
                writeUint8Array(value2);
              } else {
                var keys = Object.keys(value2);
                length = keys.length;
                writeTypeAndLength(5, length);
                for (i2 = 0; i2 < length; ++i2) {
                  var key = keys[i2];
                  encodeItem(key);
                  encodeItem(value2[key]);
                }
              }
          }
        }
        encodeItem(value);
        if ("slice" in data)
          return data.slice(0, offset);
        var ret = new ArrayBuffer(offset);
        var retView = new DataView(ret);
        for (var i = 0; i < offset; ++i)
          retView.setUint8(i, dataView.getUint8(i));
        return ret;
      }
      function decode(data, tagger, simpleValue) {
        var dataView = new DataView(data);
        var offset = 0;
        if (typeof tagger !== "function")
          tagger = function(value) {
            return value;
          };
        if (typeof simpleValue !== "function")
          simpleValue = function() {
            return undefined2;
          };
        function read(value, length) {
          offset += length;
          return value;
        }
        function readArrayBuffer(length) {
          return read(new Uint8Array(data, offset, length), length);
        }
        function readFloat16() {
          var tempArrayBuffer = new ArrayBuffer(4);
          var tempDataView = new DataView(tempArrayBuffer);
          var value = readUint16();
          var sign = value & 32768;
          var exponent = value & 31744;
          var fraction = value & 1023;
          if (exponent === 31744)
            exponent = 255 << 10;
          else if (exponent !== 0)
            exponent += 127 - 15 << 10;
          else if (fraction !== 0)
            return fraction * POW_2_24;
          tempDataView.setUint32(0, sign << 16 | exponent << 13 | fraction << 13);
          return tempDataView.getFloat32(0);
        }
        function readFloat32() {
          return read(dataView.getFloat32(offset), 4);
        }
        function readFloat64() {
          return read(dataView.getFloat64(offset), 8);
        }
        function readUint8() {
          return read(dataView.getUint8(offset), 1);
        }
        function readUint16() {
          return read(dataView.getUint16(offset), 2);
        }
        function readUint32() {
          return read(dataView.getUint32(offset), 4);
        }
        function readUint64() {
          return readUint32() * POW_2_32 + readUint32();
        }
        function readBreak() {
          if (dataView.getUint8(offset) !== 255)
            return false;
          offset += 1;
          return true;
        }
        function readLength(additionalInformation) {
          if (additionalInformation < 24)
            return additionalInformation;
          if (additionalInformation === 24)
            return readUint8();
          if (additionalInformation === 25)
            return readUint16();
          if (additionalInformation === 26)
            return readUint32();
          if (additionalInformation === 27)
            return readUint64();
          if (additionalInformation === 31)
            return -1;
          throw "Invalid length encoding";
        }
        function readIndefiniteStringLength(majorType) {
          var initialByte = readUint8();
          if (initialByte === 255)
            return -1;
          var length = readLength(initialByte & 31);
          if (length < 0 || initialByte >> 5 !== majorType)
            throw "Invalid indefinite length element";
          return length;
        }
        function appendUtf16data(utf16data, length) {
          for (var i = 0; i < length; ++i) {
            var value = readUint8();
            if (value & 128) {
              if (value < 224) {
                value = (value & 31) << 6 | readUint8() & 63;
                length -= 1;
              } else if (value < 240) {
                value = (value & 15) << 12 | (readUint8() & 63) << 6 | readUint8() & 63;
                length -= 2;
              } else {
                value = (value & 15) << 18 | (readUint8() & 63) << 12 | (readUint8() & 63) << 6 | readUint8() & 63;
                length -= 3;
              }
            }
            if (value < 65536) {
              utf16data.push(value);
            } else {
              value -= 65536;
              utf16data.push(55296 | value >> 10);
              utf16data.push(56320 | value & 1023);
            }
          }
        }
        function decodeItem() {
          var initialByte = readUint8();
          var majorType = initialByte >> 5;
          var additionalInformation = initialByte & 31;
          var i;
          var length;
          if (majorType === 7) {
            switch (additionalInformation) {
              case 25:
                return readFloat16();
              case 26:
                return readFloat32();
              case 27:
                return readFloat64();
            }
          }
          length = readLength(additionalInformation);
          if (length < 0 && (majorType < 2 || 6 < majorType))
            throw "Invalid length";
          switch (majorType) {
            case 0:
              return length;
            case 1:
              return -1 - length;
            case 2:
              if (length < 0) {
                var elements = [];
                var fullArrayLength = 0;
                while ((length = readIndefiniteStringLength(majorType)) >= 0) {
                  fullArrayLength += length;
                  elements.push(readArrayBuffer(length));
                }
                var fullArray = new Uint8Array(fullArrayLength);
                var fullArrayOffset = 0;
                for (i = 0; i < elements.length; ++i) {
                  fullArray.set(elements[i], fullArrayOffset);
                  fullArrayOffset += elements[i].length;
                }
                return fullArray;
              }
              return readArrayBuffer(length);
            case 3:
              var utf16data = [];
              if (length < 0) {
                while ((length = readIndefiniteStringLength(majorType)) >= 0)
                  appendUtf16data(utf16data, length);
              } else
                appendUtf16data(utf16data, length);
              return String.fromCharCode.apply(null, utf16data);
            case 4:
              var retArray;
              if (length < 0) {
                retArray = [];
                while (!readBreak())
                  retArray.push(decodeItem());
              } else {
                retArray = new Array(length);
                for (i = 0; i < length; ++i)
                  retArray[i] = decodeItem();
              }
              return retArray;
            case 5:
              var retObject = {};
              for (i = 0; i < length || length < 0 && !readBreak(); ++i) {
                var key = decodeItem();
                retObject[key] = decodeItem();
              }
              return retObject;
            case 6:
              return tagger(decodeItem(), length);
            case 7:
              switch (length) {
                case 20:
                  return false;
                case 21:
                  return true;
                case 22:
                  return null;
                case 23:
                  return undefined2;
                default:
                  return simpleValue(length);
              }
          }
        }
        var ret = decodeItem();
        if (offset !== data.byteLength)
          throw "Remaining bytes";
        return ret;
      }
      var obj = { encode, decode };
      if (typeof define === "function" && define.amd)
        define("cbor/cbor", obj);
      else if (typeof module !== "undefined" && module.exports)
        module.exports = obj;
      else if (!global.CBOR)
        global.CBOR = obj;
    })(exports);
  }
});

// node_modules/roslib/src/util/cborTypedArrayTags.js
var require_cborTypedArrayTags = __commonJS({
  "node_modules/roslib/src/util/cborTypedArrayTags.js"(exports, module) {
    "use strict";
    var UPPER32 = Math.pow(2, 32);
    var warnedPrecision = false;
    function warnPrecision() {
      if (!warnedPrecision) {
        warnedPrecision = true;
        console.warn("CBOR 64-bit integer array values may lose precision. No further warnings.");
      }
    }
    function decodeUint64LE(bytes) {
      warnPrecision();
      var byteLen = bytes.byteLength;
      var offset = bytes.byteOffset;
      var arrLen = byteLen / 8;
      var buffer = bytes.buffer.slice(offset, offset + byteLen);
      var uint32View = new Uint32Array(buffer);
      var arr = new Array(arrLen);
      for (var i = 0; i < arrLen; i++) {
        var si = i * 2;
        var lo = uint32View[si];
        var hi = uint32View[si + 1];
        arr[i] = lo + UPPER32 * hi;
      }
      return arr;
    }
    function decodeInt64LE(bytes) {
      warnPrecision();
      var byteLen = bytes.byteLength;
      var offset = bytes.byteOffset;
      var arrLen = byteLen / 8;
      var buffer = bytes.buffer.slice(offset, offset + byteLen);
      var uint32View = new Uint32Array(buffer);
      var int32View = new Int32Array(buffer);
      var arr = new Array(arrLen);
      for (var i = 0; i < arrLen; i++) {
        var si = i * 2;
        var lo = uint32View[si];
        var hi = int32View[si + 1];
        arr[i] = lo + UPPER32 * hi;
      }
      return arr;
    }
    function decodeNativeArray(bytes, ArrayType) {
      var byteLen = bytes.byteLength;
      var offset = bytes.byteOffset;
      var buffer = bytes.buffer.slice(offset, offset + byteLen);
      return new ArrayType(buffer);
    }
    var nativeArrayTypes = {
      64: Uint8Array,
      69: Uint16Array,
      70: Uint32Array,
      72: Int8Array,
      77: Int16Array,
      78: Int32Array,
      85: Float32Array,
      86: Float64Array
    };
    var conversionArrayTypes = {
      71: decodeUint64LE,
      79: decodeInt64LE
    };
    function cborTypedArrayTagger(data, tag) {
      if (tag in nativeArrayTypes) {
        var arrayType = nativeArrayTypes[tag];
        return decodeNativeArray(data, arrayType);
      }
      if (tag in conversionArrayTypes) {
        return conversionArrayTypes[tag](data);
      }
      return data;
    }
    if (typeof module !== "undefined" && module.exports) {
      module.exports = cborTypedArrayTagger;
    }
  }
});

// node_modules/roslib/src/core/SocketAdapter.js
var require_SocketAdapter = __commonJS({
  "node_modules/roslib/src/core/SocketAdapter.js"(exports, module) {
    "use strict";
    var decompressPng = require_decompressPng();
    var CBOR = require_cbor();
    var typedArrayTagger = require_cborTypedArrayTags();
    var BSON = null;
    if (typeof bson !== "undefined") {
      BSON = bson().BSON;
    }
    function SocketAdapter(client) {
      var decoder = null;
      if (client.transportOptions.decoder) {
        decoder = client.transportOptions.decoder;
      }
      function handleMessage(message) {
        if (message.op === "publish") {
          client.emit(message.topic, message.msg);
        } else if (message.op === "service_response") {
          client.emit(message.id, message);
        } else if (message.op === "call_service") {
          client.emit(message.service, message);
        } else if (message.op === "status") {
          if (message.id) {
            client.emit("status:" + message.id, message);
          } else {
            client.emit("status", message);
          }
        }
      }
      function handlePng(message, callback) {
        if (message.op === "png") {
          decompressPng(message.data, callback);
        } else {
          callback(message);
        }
      }
      function decodeBSON(data, callback) {
        if (!BSON) {
          throw "Cannot process BSON encoded message without BSON header.";
        }
        var reader = new FileReader();
        reader.onload = function() {
          var uint8Array = new Uint8Array(this.result);
          var msg = BSON.deserialize(uint8Array);
          callback(msg);
        };
        reader.readAsArrayBuffer(data);
      }
      return {
        /**
         * Emit a 'connection' event on WebSocket connection.
         *
         * @param {function} event - The argument to emit with the event.
         * @memberof SocketAdapter
         */
        onopen: function onOpen(event) {
          client.isConnected = true;
          client.emit("connection", event);
        },
        /**
         * Emit a 'close' event on WebSocket disconnection.
         *
         * @param {function} event - The argument to emit with the event.
         * @memberof SocketAdapter
         */
        onclose: function onClose(event) {
          client.isConnected = false;
          client.emit("close", event);
        },
        /**
         * Emit an 'error' event whenever there was an error.
         *
         * @param {function} event - The argument to emit with the event.
         * @memberof SocketAdapter
         */
        onerror: function onError(event) {
          client.emit("error", event);
        },
        /**
         * Parse message responses from rosbridge and send to the appropriate
         * topic, service, or param.
         *
         * @param {Object} data - The raw JSON message from rosbridge.
         * @memberof SocketAdapter
         */
        onmessage: function onMessage(data) {
          if (decoder) {
            decoder(data.data, function(message2) {
              handleMessage(message2);
            });
          } else if (typeof Blob !== "undefined" && data.data instanceof Blob) {
            decodeBSON(data.data, function(message2) {
              handlePng(message2, handleMessage);
            });
          } else if (data.data instanceof ArrayBuffer) {
            var decoded = CBOR.decode(data.data, typedArrayTagger);
            handleMessage(decoded);
          } else {
            var message = JSON.parse(typeof data === "string" ? data : data.data);
            handlePng(message, handleMessage);
          }
        }
      };
    }
    module.exports = SocketAdapter;
  }
});

// node_modules/roslib/src/core/ServiceResponse.js
var require_ServiceResponse = __commonJS({
  "node_modules/roslib/src/core/ServiceResponse.js"(exports, module) {
    var assign = require_object_assign();
    function ServiceResponse(values) {
      assign(this, values);
    }
    module.exports = ServiceResponse;
  }
});

// node_modules/roslib/src/core/ServiceRequest.js
var require_ServiceRequest = __commonJS({
  "node_modules/roslib/src/core/ServiceRequest.js"(exports, module) {
    var assign = require_object_assign();
    function ServiceRequest(values) {
      assign(this, values);
    }
    module.exports = ServiceRequest;
  }
});

// node_modules/eventemitter2/lib/eventemitter2.js
var require_eventemitter2 = __commonJS({
  "node_modules/eventemitter2/lib/eventemitter2.js"(exports, module) {
    !function(undefined2) {
      var hasOwnProperty = Object.hasOwnProperty;
      var isArray = Array.isArray ? Array.isArray : function _isArray(obj) {
        return Object.prototype.toString.call(obj) === "[object Array]";
      };
      var defaultMaxListeners = 10;
      var nextTickSupported = typeof process == "object" && typeof process.nextTick == "function";
      var symbolsSupported = typeof Symbol === "function";
      var reflectSupported = typeof Reflect === "object";
      var setImmediateSupported = typeof setImmediate === "function";
      var _setImmediate = setImmediateSupported ? setImmediate : setTimeout;
      var ownKeys = symbolsSupported ? reflectSupported && typeof Reflect.ownKeys === "function" ? Reflect.ownKeys : function(obj) {
        var arr = Object.getOwnPropertyNames(obj);
        arr.push.apply(arr, Object.getOwnPropertySymbols(obj));
        return arr;
      } : Object.keys;
      function init() {
        this._events = {};
        if (this._conf) {
          configure.call(this, this._conf);
        }
      }
      function configure(conf) {
        if (conf) {
          this._conf = conf;
          conf.delimiter && (this.delimiter = conf.delimiter);
          if (conf.maxListeners !== undefined2) {
            this._maxListeners = conf.maxListeners;
          }
          conf.wildcard && (this.wildcard = conf.wildcard);
          conf.newListener && (this._newListener = conf.newListener);
          conf.removeListener && (this._removeListener = conf.removeListener);
          conf.verboseMemoryLeak && (this.verboseMemoryLeak = conf.verboseMemoryLeak);
          conf.ignoreErrors && (this.ignoreErrors = conf.ignoreErrors);
          if (this.wildcard) {
            this.listenerTree = {};
          }
        }
      }
      function logPossibleMemoryLeak(count, eventName) {
        var errorMsg = "(node) warning: possible EventEmitter memory leak detected. " + count + " listeners added. Use emitter.setMaxListeners() to increase limit.";
        if (this.verboseMemoryLeak) {
          errorMsg += " Event name: " + eventName + ".";
        }
        if (typeof process !== "undefined" && process.emitWarning) {
          var e = new Error(errorMsg);
          e.name = "MaxListenersExceededWarning";
          e.emitter = this;
          e.count = count;
          process.emitWarning(e);
        } else {
          console.error(errorMsg);
          if (console.trace) {
            console.trace();
          }
        }
      }
      var toArray = function(a, b, c) {
        var n = arguments.length;
        switch (n) {
          case 0:
            return [];
          case 1:
            return [a];
          case 2:
            return [a, b];
          case 3:
            return [a, b, c];
          default:
            var arr = new Array(n);
            while (n--) {
              arr[n] = arguments[n];
            }
            return arr;
        }
      };
      function toObject(keys, values) {
        var obj = {};
        var key;
        var len = keys.length;
        var valuesCount = values ? values.length : 0;
        for (var i = 0; i < len; i++) {
          key = keys[i];
          obj[key] = i < valuesCount ? values[i] : undefined2;
        }
        return obj;
      }
      function TargetObserver(emitter, target, options) {
        this._emitter = emitter;
        this._target = target;
        this._listeners = {};
        this._listenersCount = 0;
        var on, off;
        if (options.on || options.off) {
          on = options.on;
          off = options.off;
        }
        if (target.addEventListener) {
          on = target.addEventListener;
          off = target.removeEventListener;
        } else if (target.addListener) {
          on = target.addListener;
          off = target.removeListener;
        } else if (target.on) {
          on = target.on;
          off = target.off;
        }
        if (!on && !off) {
          throw Error("target does not implement any known event API");
        }
        if (typeof on !== "function") {
          throw TypeError("on method must be a function");
        }
        if (typeof off !== "function") {
          throw TypeError("off method must be a function");
        }
        this._on = on;
        this._off = off;
        var _observers = emitter._observers;
        if (_observers) {
          _observers.push(this);
        } else {
          emitter._observers = [this];
        }
      }
      Object.assign(TargetObserver.prototype, {
        subscribe: function(event, localEvent, reducer) {
          var observer = this;
          var target = this._target;
          var emitter = this._emitter;
          var listeners = this._listeners;
          var handler = function() {
            var args = toArray.apply(null, arguments);
            var eventObj = {
              data: args,
              name: localEvent,
              original: event
            };
            if (reducer) {
              var result = reducer.call(target, eventObj);
              if (result !== false) {
                emitter.emit.apply(emitter, [eventObj.name].concat(args));
              }
              return;
            }
            emitter.emit.apply(emitter, [localEvent].concat(args));
          };
          if (listeners[event]) {
            throw Error("Event '" + event + "' is already listening");
          }
          this._listenersCount++;
          if (emitter._newListener && emitter._removeListener && !observer._onNewListener) {
            this._onNewListener = function(_event) {
              if (_event === localEvent && listeners[event] === null) {
                listeners[event] = handler;
                observer._on.call(target, event, handler);
              }
            };
            emitter.on("newListener", this._onNewListener);
            this._onRemoveListener = function(_event) {
              if (_event === localEvent && !emitter.hasListeners(_event) && listeners[event]) {
                listeners[event] = null;
                observer._off.call(target, event, handler);
              }
            };
            listeners[event] = null;
            emitter.on("removeListener", this._onRemoveListener);
          } else {
            listeners[event] = handler;
            observer._on.call(target, event, handler);
          }
        },
        unsubscribe: function(event) {
          var observer = this;
          var listeners = this._listeners;
          var emitter = this._emitter;
          var handler;
          var events;
          var off = this._off;
          var target = this._target;
          var i;
          if (event && typeof event !== "string") {
            throw TypeError("event must be a string");
          }
          function clearRefs() {
            if (observer._onNewListener) {
              emitter.off("newListener", observer._onNewListener);
              emitter.off("removeListener", observer._onRemoveListener);
              observer._onNewListener = null;
              observer._onRemoveListener = null;
            }
            var index = findTargetIndex.call(emitter, observer);
            emitter._observers.splice(index, 1);
          }
          if (event) {
            handler = listeners[event];
            if (!handler) return;
            off.call(target, event, handler);
            delete listeners[event];
            if (!--this._listenersCount) {
              clearRefs();
            }
          } else {
            events = ownKeys(listeners);
            i = events.length;
            while (i-- > 0) {
              event = events[i];
              off.call(target, event, listeners[event]);
            }
            this._listeners = {};
            this._listenersCount = 0;
            clearRefs();
          }
        }
      });
      function resolveOptions(options, schema, reducers, allowUnknown) {
        var computedOptions = Object.assign({}, schema);
        if (!options) return computedOptions;
        if (typeof options !== "object") {
          throw TypeError("options must be an object");
        }
        var keys = Object.keys(options);
        var length = keys.length;
        var option, value;
        var reducer;
        function reject(reason) {
          throw Error('Invalid "' + option + '" option value' + (reason ? ". Reason: " + reason : ""));
        }
        for (var i = 0; i < length; i++) {
          option = keys[i];
          if (!allowUnknown && !hasOwnProperty.call(schema, option)) {
            throw Error('Unknown "' + option + '" option');
          }
          value = options[option];
          if (value !== undefined2) {
            reducer = reducers[option];
            computedOptions[option] = reducer ? reducer(value, reject) : value;
          }
        }
        return computedOptions;
      }
      function constructorReducer(value, reject) {
        if (typeof value !== "function" || !value.hasOwnProperty("prototype")) {
          reject("value must be a constructor");
        }
        return value;
      }
      function makeTypeReducer(types) {
        var message = "value must be type of " + types.join("|");
        var len = types.length;
        var firstType = types[0];
        var secondType = types[1];
        if (len === 1) {
          return function(v, reject) {
            if (typeof v === firstType) {
              return v;
            }
            reject(message);
          };
        }
        if (len === 2) {
          return function(v, reject) {
            var kind = typeof v;
            if (kind === firstType || kind === secondType) return v;
            reject(message);
          };
        }
        return function(v, reject) {
          var kind = typeof v;
          var i = len;
          while (i-- > 0) {
            if (kind === types[i]) return v;
          }
          reject(message);
        };
      }
      var functionReducer = makeTypeReducer(["function"]);
      var objectFunctionReducer = makeTypeReducer(["object", "function"]);
      function makeCancelablePromise(Promise2, executor, options) {
        var isCancelable;
        var callbacks;
        var timer = 0;
        var subscriptionClosed;
        var promise = new Promise2(function(resolve, reject, onCancel) {
          options = resolveOptions(options, {
            timeout: 0,
            overload: false
          }, {
            timeout: function(value, reject2) {
              value *= 1;
              if (typeof value !== "number" || value < 0 || !Number.isFinite(value)) {
                reject2("timeout must be a positive number");
              }
              return value;
            }
          });
          isCancelable = !options.overload && typeof Promise2.prototype.cancel === "function" && typeof onCancel === "function";
          function cleanup() {
            if (callbacks) {
              callbacks = null;
            }
            if (timer) {
              clearTimeout(timer);
              timer = 0;
            }
          }
          var _resolve = function(value) {
            cleanup();
            resolve(value);
          };
          var _reject = function(err) {
            cleanup();
            reject(err);
          };
          if (isCancelable) {
            executor(_resolve, _reject, onCancel);
          } else {
            callbacks = [function(reason) {
              _reject(reason || Error("canceled"));
            }];
            executor(_resolve, _reject, function(cb) {
              if (subscriptionClosed) {
                throw Error("Unable to subscribe on cancel event asynchronously");
              }
              if (typeof cb !== "function") {
                throw TypeError("onCancel callback must be a function");
              }
              callbacks.push(cb);
            });
            subscriptionClosed = true;
          }
          if (options.timeout > 0) {
            timer = setTimeout(function() {
              var reason = Error("timeout");
              reason.code = "ETIMEDOUT";
              timer = 0;
              promise.cancel(reason);
              reject(reason);
            }, options.timeout);
          }
        });
        if (!isCancelable) {
          promise.cancel = function(reason) {
            if (!callbacks) {
              return;
            }
            var length = callbacks.length;
            for (var i = 1; i < length; i++) {
              callbacks[i](reason);
            }
            callbacks[0](reason);
            callbacks = null;
          };
        }
        return promise;
      }
      function findTargetIndex(observer) {
        var observers = this._observers;
        if (!observers) {
          return -1;
        }
        var len = observers.length;
        for (var i = 0; i < len; i++) {
          if (observers[i]._target === observer) return i;
        }
        return -1;
      }
      function searchListenerTree(handlers, type, tree, i, typeLength) {
        if (!tree) {
          return null;
        }
        if (i === 0) {
          var kind = typeof type;
          if (kind === "string") {
            var ns, n, l = 0, j = 0, delimiter = this.delimiter, dl = delimiter.length;
            if ((n = type.indexOf(delimiter)) !== -1) {
              ns = new Array(5);
              do {
                ns[l++] = type.slice(j, n);
                j = n + dl;
              } while ((n = type.indexOf(delimiter, j)) !== -1);
              ns[l++] = type.slice(j);
              type = ns;
              typeLength = l;
            } else {
              type = [type];
              typeLength = 1;
            }
          } else if (kind === "object") {
            typeLength = type.length;
          } else {
            type = [type];
            typeLength = 1;
          }
        }
        var listeners = null, branch, xTree, xxTree, isolatedBranch, endReached, currentType = type[i], nextType = type[i + 1], branches, _listeners;
        if (i === typeLength) {
          if (tree._listeners) {
            if (typeof tree._listeners === "function") {
              handlers && handlers.push(tree._listeners);
              listeners = [tree];
            } else {
              handlers && handlers.push.apply(handlers, tree._listeners);
              listeners = [tree];
            }
          }
        } else {
          if (currentType === "*") {
            branches = ownKeys(tree);
            n = branches.length;
            while (n-- > 0) {
              branch = branches[n];
              if (branch !== "_listeners") {
                _listeners = searchListenerTree(handlers, type, tree[branch], i + 1, typeLength);
                if (_listeners) {
                  if (listeners) {
                    listeners.push.apply(listeners, _listeners);
                  } else {
                    listeners = _listeners;
                  }
                }
              }
            }
            return listeners;
          } else if (currentType === "**") {
            endReached = i + 1 === typeLength || i + 2 === typeLength && nextType === "*";
            if (endReached && tree._listeners) {
              listeners = searchListenerTree(handlers, type, tree, typeLength, typeLength);
            }
            branches = ownKeys(tree);
            n = branches.length;
            while (n-- > 0) {
              branch = branches[n];
              if (branch !== "_listeners") {
                if (branch === "*" || branch === "**") {
                  if (tree[branch]._listeners && !endReached) {
                    _listeners = searchListenerTree(handlers, type, tree[branch], typeLength, typeLength);
                    if (_listeners) {
                      if (listeners) {
                        listeners.push.apply(listeners, _listeners);
                      } else {
                        listeners = _listeners;
                      }
                    }
                  }
                  _listeners = searchListenerTree(handlers, type, tree[branch], i, typeLength);
                } else if (branch === nextType) {
                  _listeners = searchListenerTree(handlers, type, tree[branch], i + 2, typeLength);
                } else {
                  _listeners = searchListenerTree(handlers, type, tree[branch], i, typeLength);
                }
                if (_listeners) {
                  if (listeners) {
                    listeners.push.apply(listeners, _listeners);
                  } else {
                    listeners = _listeners;
                  }
                }
              }
            }
            return listeners;
          } else if (tree[currentType]) {
            listeners = searchListenerTree(handlers, type, tree[currentType], i + 1, typeLength);
          }
        }
        xTree = tree["*"];
        if (xTree) {
          searchListenerTree(handlers, type, xTree, i + 1, typeLength);
        }
        xxTree = tree["**"];
        if (xxTree) {
          if (i < typeLength) {
            if (xxTree._listeners) {
              searchListenerTree(handlers, type, xxTree, typeLength, typeLength);
            }
            branches = ownKeys(xxTree);
            n = branches.length;
            while (n-- > 0) {
              branch = branches[n];
              if (branch !== "_listeners") {
                if (branch === nextType) {
                  searchListenerTree(handlers, type, xxTree[branch], i + 2, typeLength);
                } else if (branch === currentType) {
                  searchListenerTree(handlers, type, xxTree[branch], i + 1, typeLength);
                } else {
                  isolatedBranch = {};
                  isolatedBranch[branch] = xxTree[branch];
                  searchListenerTree(handlers, type, { "**": isolatedBranch }, i + 1, typeLength);
                }
              }
            }
          } else if (xxTree._listeners) {
            searchListenerTree(handlers, type, xxTree, typeLength, typeLength);
          } else if (xxTree["*"] && xxTree["*"]._listeners) {
            searchListenerTree(handlers, type, xxTree["*"], typeLength, typeLength);
          }
        }
        return listeners;
      }
      function growListenerTree(type, listener, prepend) {
        var len = 0, j = 0, i, delimiter = this.delimiter, dl = delimiter.length, ns;
        if (typeof type === "string") {
          if ((i = type.indexOf(delimiter)) !== -1) {
            ns = new Array(5);
            do {
              ns[len++] = type.slice(j, i);
              j = i + dl;
            } while ((i = type.indexOf(delimiter, j)) !== -1);
            ns[len++] = type.slice(j);
          } else {
            ns = [type];
            len = 1;
          }
        } else {
          ns = type;
          len = type.length;
        }
        if (len > 1) {
          for (i = 0; i + 1 < len; i++) {
            if (ns[i] === "**" && ns[i + 1] === "**") {
              return;
            }
          }
        }
        var tree = this.listenerTree, name;
        for (i = 0; i < len; i++) {
          name = ns[i];
          tree = tree[name] || (tree[name] = {});
          if (i === len - 1) {
            if (!tree._listeners) {
              tree._listeners = listener;
            } else {
              if (typeof tree._listeners === "function") {
                tree._listeners = [tree._listeners];
              }
              if (prepend) {
                tree._listeners.unshift(listener);
              } else {
                tree._listeners.push(listener);
              }
              if (!tree._listeners.warned && this._maxListeners > 0 && tree._listeners.length > this._maxListeners) {
                tree._listeners.warned = true;
                logPossibleMemoryLeak.call(this, tree._listeners.length, name);
              }
            }
            return true;
          }
        }
        return true;
      }
      function collectTreeEvents(tree, events, root, asArray) {
        var branches = ownKeys(tree);
        var i = branches.length;
        var branch, branchName, path;
        var hasListeners = tree["_listeners"];
        var isArrayPath;
        while (i-- > 0) {
          branchName = branches[i];
          branch = tree[branchName];
          if (branchName === "_listeners") {
            path = root;
          } else {
            path = root ? root.concat(branchName) : [branchName];
          }
          isArrayPath = asArray || typeof branchName === "symbol";
          hasListeners && events.push(isArrayPath ? path : path.join(this.delimiter));
          if (typeof branch === "object") {
            collectTreeEvents.call(this, branch, events, path, isArrayPath);
          }
        }
        return events;
      }
      function recursivelyGarbageCollect(root) {
        var keys = ownKeys(root);
        var i = keys.length;
        var obj, key, flag;
        while (i-- > 0) {
          key = keys[i];
          obj = root[key];
          if (obj) {
            flag = true;
            if (key !== "_listeners" && !recursivelyGarbageCollect(obj)) {
              delete root[key];
            }
          }
        }
        return flag;
      }
      function Listener(emitter, event, listener) {
        this.emitter = emitter;
        this.event = event;
        this.listener = listener;
      }
      Listener.prototype.off = function() {
        this.emitter.off(this.event, this.listener);
        return this;
      };
      function setupListener(event, listener, options) {
        if (options === true) {
          promisify = true;
        } else if (options === false) {
          async = true;
        } else {
          if (!options || typeof options !== "object") {
            throw TypeError("options should be an object or true");
          }
          var async = options.async;
          var promisify = options.promisify;
          var nextTick = options.nextTick;
          var objectify = options.objectify;
        }
        if (async || nextTick || promisify) {
          var _listener = listener;
          var _origin = listener._origin || listener;
          if (nextTick && !nextTickSupported) {
            throw Error("process.nextTick is not supported");
          }
          if (promisify === undefined2) {
            promisify = listener.constructor.name === "AsyncFunction";
          }
          listener = function() {
            var args = arguments;
            var context = this;
            var event2 = this.event;
            return promisify ? nextTick ? Promise.resolve() : new Promise(function(resolve) {
              _setImmediate(resolve);
            }).then(function() {
              context.event = event2;
              return _listener.apply(context, args);
            }) : (nextTick ? process.nextTick : _setImmediate)(function() {
              context.event = event2;
              _listener.apply(context, args);
            });
          };
          listener._async = true;
          listener._origin = _origin;
        }
        return [listener, objectify ? new Listener(this, event, listener) : this];
      }
      function EventEmitter(conf) {
        this._events = {};
        this._newListener = false;
        this._removeListener = false;
        this.verboseMemoryLeak = false;
        configure.call(this, conf);
      }
      EventEmitter.EventEmitter2 = EventEmitter;
      EventEmitter.prototype.listenTo = function(target, events, options) {
        if (typeof target !== "object") {
          throw TypeError("target musts be an object");
        }
        var emitter = this;
        options = resolveOptions(options, {
          on: undefined2,
          off: undefined2,
          reducers: undefined2
        }, {
          on: functionReducer,
          off: functionReducer,
          reducers: objectFunctionReducer
        });
        function listen(events2) {
          if (typeof events2 !== "object") {
            throw TypeError("events must be an object");
          }
          var reducers = options.reducers;
          var index = findTargetIndex.call(emitter, target);
          var observer;
          if (index === -1) {
            observer = new TargetObserver(emitter, target, options);
          } else {
            observer = emitter._observers[index];
          }
          var keys = ownKeys(events2);
          var len = keys.length;
          var event;
          var isSingleReducer = typeof reducers === "function";
          for (var i = 0; i < len; i++) {
            event = keys[i];
            observer.subscribe(
              event,
              events2[event] || event,
              isSingleReducer ? reducers : reducers && reducers[event]
            );
          }
        }
        isArray(events) ? listen(toObject(events)) : typeof events === "string" ? listen(toObject(events.split(/\s+/))) : listen(events);
        return this;
      };
      EventEmitter.prototype.stopListeningTo = function(target, event) {
        var observers = this._observers;
        if (!observers) {
          return false;
        }
        var i = observers.length;
        var observer;
        var matched = false;
        if (target && typeof target !== "object") {
          throw TypeError("target should be an object");
        }
        while (i-- > 0) {
          observer = observers[i];
          if (!target || observer._target === target) {
            observer.unsubscribe(event);
            matched = true;
          }
        }
        return matched;
      };
      EventEmitter.prototype.delimiter = ".";
      EventEmitter.prototype.setMaxListeners = function(n) {
        if (n !== undefined2) {
          this._maxListeners = n;
          if (!this._conf) this._conf = {};
          this._conf.maxListeners = n;
        }
      };
      EventEmitter.prototype.getMaxListeners = function() {
        return this._maxListeners;
      };
      EventEmitter.prototype.event = "";
      EventEmitter.prototype.once = function(event, fn, options) {
        return this._once(event, fn, false, options);
      };
      EventEmitter.prototype.prependOnceListener = function(event, fn, options) {
        return this._once(event, fn, true, options);
      };
      EventEmitter.prototype._once = function(event, fn, prepend, options) {
        return this._many(event, 1, fn, prepend, options);
      };
      EventEmitter.prototype.many = function(event, ttl, fn, options) {
        return this._many(event, ttl, fn, false, options);
      };
      EventEmitter.prototype.prependMany = function(event, ttl, fn, options) {
        return this._many(event, ttl, fn, true, options);
      };
      EventEmitter.prototype._many = function(event, ttl, fn, prepend, options) {
        var self = this;
        if (typeof fn !== "function") {
          throw new Error("many only accepts instances of Function");
        }
        function listener() {
          if (--ttl === 0) {
            self.off(event, listener);
          }
          return fn.apply(this, arguments);
        }
        listener._origin = fn;
        return this._on(event, listener, prepend, options);
      };
      EventEmitter.prototype.emit = function() {
        if (!this._events && !this._all) {
          return false;
        }
        this._events || init.call(this);
        var type = arguments[0], ns, wildcard = this.wildcard;
        var args, l, i, j, containsSymbol;
        if (type === "newListener" && !this._newListener) {
          if (!this._events.newListener) {
            return false;
          }
        }
        if (wildcard) {
          ns = type;
          if (type !== "newListener" && type !== "removeListener") {
            if (typeof type === "object") {
              l = type.length;
              if (symbolsSupported) {
                for (i = 0; i < l; i++) {
                  if (typeof type[i] === "symbol") {
                    containsSymbol = true;
                    break;
                  }
                }
              }
              if (!containsSymbol) {
                type = type.join(this.delimiter);
              }
            }
          }
        }
        var al = arguments.length;
        var handler;
        if (this._all && this._all.length) {
          handler = this._all.slice();
          for (i = 0, l = handler.length; i < l; i++) {
            this.event = type;
            switch (al) {
              case 1:
                handler[i].call(this, type);
                break;
              case 2:
                handler[i].call(this, type, arguments[1]);
                break;
              case 3:
                handler[i].call(this, type, arguments[1], arguments[2]);
                break;
              default:
                handler[i].apply(this, arguments);
            }
          }
        }
        if (wildcard) {
          handler = [];
          searchListenerTree.call(this, handler, ns, this.listenerTree, 0, l);
        } else {
          handler = this._events[type];
          if (typeof handler === "function") {
            this.event = type;
            switch (al) {
              case 1:
                handler.call(this);
                break;
              case 2:
                handler.call(this, arguments[1]);
                break;
              case 3:
                handler.call(this, arguments[1], arguments[2]);
                break;
              default:
                args = new Array(al - 1);
                for (j = 1; j < al; j++) args[j - 1] = arguments[j];
                handler.apply(this, args);
            }
            return true;
          } else if (handler) {
            handler = handler.slice();
          }
        }
        if (handler && handler.length) {
          if (al > 3) {
            args = new Array(al - 1);
            for (j = 1; j < al; j++) args[j - 1] = arguments[j];
          }
          for (i = 0, l = handler.length; i < l; i++) {
            this.event = type;
            switch (al) {
              case 1:
                handler[i].call(this);
                break;
              case 2:
                handler[i].call(this, arguments[1]);
                break;
              case 3:
                handler[i].call(this, arguments[1], arguments[2]);
                break;
              default:
                handler[i].apply(this, args);
            }
          }
          return true;
        } else if (!this.ignoreErrors && !this._all && type === "error") {
          if (arguments[1] instanceof Error) {
            throw arguments[1];
          } else {
            throw new Error("Uncaught, unspecified 'error' event.");
          }
        }
        return !!this._all;
      };
      EventEmitter.prototype.emitAsync = function() {
        if (!this._events && !this._all) {
          return false;
        }
        this._events || init.call(this);
        var type = arguments[0], wildcard = this.wildcard, ns, containsSymbol;
        var args, l, i, j;
        if (type === "newListener" && !this._newListener) {
          if (!this._events.newListener) {
            return Promise.resolve([false]);
          }
        }
        if (wildcard) {
          ns = type;
          if (type !== "newListener" && type !== "removeListener") {
            if (typeof type === "object") {
              l = type.length;
              if (symbolsSupported) {
                for (i = 0; i < l; i++) {
                  if (typeof type[i] === "symbol") {
                    containsSymbol = true;
                    break;
                  }
                }
              }
              if (!containsSymbol) {
                type = type.join(this.delimiter);
              }
            }
          }
        }
        var promises = [];
        var al = arguments.length;
        var handler;
        if (this._all) {
          for (i = 0, l = this._all.length; i < l; i++) {
            this.event = type;
            switch (al) {
              case 1:
                promises.push(this._all[i].call(this, type));
                break;
              case 2:
                promises.push(this._all[i].call(this, type, arguments[1]));
                break;
              case 3:
                promises.push(this._all[i].call(this, type, arguments[1], arguments[2]));
                break;
              default:
                promises.push(this._all[i].apply(this, arguments));
            }
          }
        }
        if (wildcard) {
          handler = [];
          searchListenerTree.call(this, handler, ns, this.listenerTree, 0);
        } else {
          handler = this._events[type];
        }
        if (typeof handler === "function") {
          this.event = type;
          switch (al) {
            case 1:
              promises.push(handler.call(this));
              break;
            case 2:
              promises.push(handler.call(this, arguments[1]));
              break;
            case 3:
              promises.push(handler.call(this, arguments[1], arguments[2]));
              break;
            default:
              args = new Array(al - 1);
              for (j = 1; j < al; j++) args[j - 1] = arguments[j];
              promises.push(handler.apply(this, args));
          }
        } else if (handler && handler.length) {
          handler = handler.slice();
          if (al > 3) {
            args = new Array(al - 1);
            for (j = 1; j < al; j++) args[j - 1] = arguments[j];
          }
          for (i = 0, l = handler.length; i < l; i++) {
            this.event = type;
            switch (al) {
              case 1:
                promises.push(handler[i].call(this));
                break;
              case 2:
                promises.push(handler[i].call(this, arguments[1]));
                break;
              case 3:
                promises.push(handler[i].call(this, arguments[1], arguments[2]));
                break;
              default:
                promises.push(handler[i].apply(this, args));
            }
          }
        } else if (!this.ignoreErrors && !this._all && type === "error") {
          if (arguments[1] instanceof Error) {
            return Promise.reject(arguments[1]);
          } else {
            return Promise.reject("Uncaught, unspecified 'error' event.");
          }
        }
        return Promise.all(promises);
      };
      EventEmitter.prototype.on = function(type, listener, options) {
        return this._on(type, listener, false, options);
      };
      EventEmitter.prototype.prependListener = function(type, listener, options) {
        return this._on(type, listener, true, options);
      };
      EventEmitter.prototype.onAny = function(fn) {
        return this._onAny(fn, false);
      };
      EventEmitter.prototype.prependAny = function(fn) {
        return this._onAny(fn, true);
      };
      EventEmitter.prototype.addListener = EventEmitter.prototype.on;
      EventEmitter.prototype._onAny = function(fn, prepend) {
        if (typeof fn !== "function") {
          throw new Error("onAny only accepts instances of Function");
        }
        if (!this._all) {
          this._all = [];
        }
        if (prepend) {
          this._all.unshift(fn);
        } else {
          this._all.push(fn);
        }
        return this;
      };
      EventEmitter.prototype._on = function(type, listener, prepend, options) {
        if (typeof type === "function") {
          this._onAny(type, listener);
          return this;
        }
        if (typeof listener !== "function") {
          throw new Error("on only accepts instances of Function");
        }
        this._events || init.call(this);
        var returnValue = this, temp;
        if (options !== undefined2) {
          temp = setupListener.call(this, type, listener, options);
          listener = temp[0];
          returnValue = temp[1];
        }
        if (this._newListener) {
          this.emit("newListener", type, listener);
        }
        if (this.wildcard) {
          growListenerTree.call(this, type, listener, prepend);
          return returnValue;
        }
        if (!this._events[type]) {
          this._events[type] = listener;
        } else {
          if (typeof this._events[type] === "function") {
            this._events[type] = [this._events[type]];
          }
          if (prepend) {
            this._events[type].unshift(listener);
          } else {
            this._events[type].push(listener);
          }
          if (!this._events[type].warned && this._maxListeners > 0 && this._events[type].length > this._maxListeners) {
            this._events[type].warned = true;
            logPossibleMemoryLeak.call(this, this._events[type].length, type);
          }
        }
        return returnValue;
      };
      EventEmitter.prototype.off = function(type, listener) {
        if (typeof listener !== "function") {
          throw new Error("removeListener only takes instances of Function");
        }
        var handlers, leafs = [];
        if (this.wildcard) {
          var ns = typeof type === "string" ? type.split(this.delimiter) : type.slice();
          leafs = searchListenerTree.call(this, null, ns, this.listenerTree, 0);
          if (!leafs) return this;
        } else {
          if (!this._events[type]) return this;
          handlers = this._events[type];
          leafs.push({ _listeners: handlers });
        }
        for (var iLeaf = 0; iLeaf < leafs.length; iLeaf++) {
          var leaf = leafs[iLeaf];
          handlers = leaf._listeners;
          if (isArray(handlers)) {
            var position = -1;
            for (var i = 0, length = handlers.length; i < length; i++) {
              if (handlers[i] === listener || handlers[i].listener && handlers[i].listener === listener || handlers[i]._origin && handlers[i]._origin === listener) {
                position = i;
                break;
              }
            }
            if (position < 0) {
              continue;
            }
            if (this.wildcard) {
              leaf._listeners.splice(position, 1);
            } else {
              this._events[type].splice(position, 1);
            }
            if (handlers.length === 0) {
              if (this.wildcard) {
                delete leaf._listeners;
              } else {
                delete this._events[type];
              }
            }
            if (this._removeListener)
              this.emit("removeListener", type, listener);
            return this;
          } else if (handlers === listener || handlers.listener && handlers.listener === listener || handlers._origin && handlers._origin === listener) {
            if (this.wildcard) {
              delete leaf._listeners;
            } else {
              delete this._events[type];
            }
            if (this._removeListener)
              this.emit("removeListener", type, listener);
          }
        }
        this.listenerTree && recursivelyGarbageCollect(this.listenerTree);
        return this;
      };
      EventEmitter.prototype.offAny = function(fn) {
        var i = 0, l = 0, fns;
        if (fn && this._all && this._all.length > 0) {
          fns = this._all;
          for (i = 0, l = fns.length; i < l; i++) {
            if (fn === fns[i]) {
              fns.splice(i, 1);
              if (this._removeListener)
                this.emit("removeListenerAny", fn);
              return this;
            }
          }
        } else {
          fns = this._all;
          if (this._removeListener) {
            for (i = 0, l = fns.length; i < l; i++)
              this.emit("removeListenerAny", fns[i]);
          }
          this._all = [];
        }
        return this;
      };
      EventEmitter.prototype.removeListener = EventEmitter.prototype.off;
      EventEmitter.prototype.removeAllListeners = function(type) {
        if (type === undefined2) {
          !this._events || init.call(this);
          return this;
        }
        if (this.wildcard) {
          var leafs = searchListenerTree.call(this, null, type, this.listenerTree, 0), leaf, i;
          if (!leafs) return this;
          for (i = 0; i < leafs.length; i++) {
            leaf = leafs[i];
            leaf._listeners = null;
          }
          this.listenerTree && recursivelyGarbageCollect(this.listenerTree);
        } else if (this._events) {
          this._events[type] = null;
        }
        return this;
      };
      EventEmitter.prototype.listeners = function(type) {
        var _events = this._events;
        var keys, listeners, allListeners;
        var i;
        var listenerTree;
        if (type === undefined2) {
          if (this.wildcard) {
            throw Error("event name required for wildcard emitter");
          }
          if (!_events) {
            return [];
          }
          keys = ownKeys(_events);
          i = keys.length;
          allListeners = [];
          while (i-- > 0) {
            listeners = _events[keys[i]];
            if (typeof listeners === "function") {
              allListeners.push(listeners);
            } else {
              allListeners.push.apply(allListeners, listeners);
            }
          }
          return allListeners;
        } else {
          if (this.wildcard) {
            listenerTree = this.listenerTree;
            if (!listenerTree) return [];
            var handlers = [];
            var ns = typeof type === "string" ? type.split(this.delimiter) : type.slice();
            searchListenerTree.call(this, handlers, ns, listenerTree, 0);
            return handlers;
          }
          if (!_events) {
            return [];
          }
          listeners = _events[type];
          if (!listeners) {
            return [];
          }
          return typeof listeners === "function" ? [listeners] : listeners;
        }
      };
      EventEmitter.prototype.eventNames = function(nsAsArray) {
        var _events = this._events;
        return this.wildcard ? collectTreeEvents.call(this, this.listenerTree, [], null, nsAsArray) : _events ? ownKeys(_events) : [];
      };
      EventEmitter.prototype.listenerCount = function(type) {
        return this.listeners(type).length;
      };
      EventEmitter.prototype.hasListeners = function(type) {
        if (this.wildcard) {
          var handlers = [];
          var ns = typeof type === "string" ? type.split(this.delimiter) : type.slice();
          searchListenerTree.call(this, handlers, ns, this.listenerTree, 0);
          return handlers.length > 0;
        }
        var _events = this._events;
        var _all = this._all;
        return !!(_all && _all.length || _events && (type === undefined2 ? ownKeys(_events).length : _events[type]));
      };
      EventEmitter.prototype.listenersAny = function() {
        if (this._all) {
          return this._all;
        } else {
          return [];
        }
      };
      EventEmitter.prototype.waitFor = function(event, options) {
        var self = this;
        var type = typeof options;
        if (type === "number") {
          options = { timeout: options };
        } else if (type === "function") {
          options = { filter: options };
        }
        options = resolveOptions(options, {
          timeout: 0,
          filter: undefined2,
          handleError: false,
          Promise,
          overload: false
        }, {
          filter: functionReducer,
          Promise: constructorReducer
        });
        return makeCancelablePromise(options.Promise, function(resolve, reject, onCancel) {
          function listener() {
            var filter = options.filter;
            if (filter && !filter.apply(self, arguments)) {
              return;
            }
            self.off(event, listener);
            if (options.handleError) {
              var err = arguments[0];
              err ? reject(err) : resolve(toArray.apply(null, arguments).slice(1));
            } else {
              resolve(toArray.apply(null, arguments));
            }
          }
          onCancel(function() {
            self.off(event, listener);
          });
          self._on(event, listener, false);
        }, {
          timeout: options.timeout,
          overload: options.overload
        });
      };
      function once(emitter, name, options) {
        options = resolveOptions(options, {
          Promise,
          timeout: 0,
          overload: false
        }, {
          Promise: constructorReducer
        });
        var _Promise = options.Promise;
        return makeCancelablePromise(_Promise, function(resolve, reject, onCancel) {
          var handler;
          if (typeof emitter.addEventListener === "function") {
            handler = function() {
              resolve(toArray.apply(null, arguments));
            };
            onCancel(function() {
              emitter.removeEventListener(name, handler);
            });
            emitter.addEventListener(
              name,
              handler,
              { once: true }
            );
            return;
          }
          var eventListener = function() {
            errorListener && emitter.removeListener("error", errorListener);
            resolve(toArray.apply(null, arguments));
          };
          var errorListener;
          if (name !== "error") {
            errorListener = function(err) {
              emitter.removeListener(name, eventListener);
              reject(err);
            };
            emitter.once("error", errorListener);
          }
          onCancel(function() {
            errorListener && emitter.removeListener("error", errorListener);
            emitter.removeListener(name, eventListener);
          });
          emitter.once(name, eventListener);
        }, {
          timeout: options.timeout,
          overload: options.overload
        });
      }
      var prototype = EventEmitter.prototype;
      Object.defineProperties(EventEmitter, {
        defaultMaxListeners: {
          get: function() {
            return prototype._maxListeners;
          },
          set: function(n) {
            if (typeof n !== "number" || n < 0 || Number.isNaN(n)) {
              throw TypeError("n must be a non-negative number");
            }
            prototype._maxListeners = n;
          },
          enumerable: true
        },
        once: {
          value: once,
          writable: true,
          configurable: true
        }
      });
      Object.defineProperties(prototype, {
        _maxListeners: {
          value: defaultMaxListeners,
          writable: true,
          configurable: true
        },
        _observers: { value: null, writable: true, configurable: true }
      });
      if (typeof define === "function" && define.amd) {
        define(function() {
          return EventEmitter;
        });
      } else if (typeof exports === "object") {
        module.exports = EventEmitter;
      } else {
        var _global = new Function("", "return this")();
        _global.EventEmitter2 = EventEmitter;
      }
    }();
  }
});

// node_modules/roslib/src/core/Service.js
var require_Service = __commonJS({
  "node_modules/roslib/src/core/Service.js"(exports, module) {
    var ServiceResponse = require_ServiceResponse();
    var ServiceRequest = require_ServiceRequest();
    var EventEmitter2 = require_eventemitter2().EventEmitter2;
    function Service(options) {
      options = options || {};
      this.ros = options.ros;
      this.name = options.name;
      this.serviceType = options.serviceType;
      this.isAdvertised = false;
      this._serviceCallback = null;
    }
    Service.prototype.__proto__ = EventEmitter2.prototype;
    Service.prototype.callService = function(request, callback, failedCallback) {
      if (this.isAdvertised) {
        return;
      }
      var serviceCallId = "call_service:" + this.name + ":" + ++this.ros.idCounter;
      if (callback || failedCallback) {
        this.ros.once(serviceCallId, function(message) {
          if (message.result !== void 0 && message.result === false) {
            if (typeof failedCallback === "function") {
              failedCallback(message.values);
            }
          } else if (typeof callback === "function") {
            callback(new ServiceResponse(message.values));
          }
        });
      }
      var call = {
        op: "call_service",
        id: serviceCallId,
        service: this.name,
        type: this.serviceType,
        args: request
      };
      this.ros.callOnConnection(call);
    };
    Service.prototype.advertise = function(callback) {
      if (this.isAdvertised || typeof callback !== "function") {
        return;
      }
      this._serviceCallback = callback;
      this.ros.on(this.name, this._serviceResponse.bind(this));
      this.ros.callOnConnection({
        op: "advertise_service",
        type: this.serviceType,
        service: this.name
      });
      this.isAdvertised = true;
    };
    Service.prototype.unadvertise = function() {
      if (!this.isAdvertised) {
        return;
      }
      this.ros.callOnConnection({
        op: "unadvertise_service",
        service: this.name
      });
      this.isAdvertised = false;
    };
    Service.prototype._serviceResponse = function(rosbridgeRequest) {
      var response = {};
      var success = this._serviceCallback(rosbridgeRequest.args, response);
      var call = {
        op: "service_response",
        service: this.name,
        values: new ServiceResponse(response),
        result: success
      };
      if (rosbridgeRequest.id) {
        call.id = rosbridgeRequest.id;
      }
      this.ros.callOnConnection(call);
    };
    module.exports = Service;
  }
});

// node_modules/roslib/src/core/Ros.js
var require_Ros = __commonJS({
  "node_modules/roslib/src/core/Ros.js"(exports, module) {
    var WebSocket2 = require_WebSocket();
    var WorkerSocket = require_workerSocket();
    var socketAdapter = require_SocketAdapter();
    var Service = require_Service();
    var ServiceRequest = require_ServiceRequest();
    var assign = require_object_assign();
    var EventEmitter2 = require_eventemitter2().EventEmitter2;
    function Ros(options) {
      options = options || {};
      var that = this;
      this.socket = null;
      this.idCounter = 0;
      this.isConnected = false;
      this.transportLibrary = options.transportLibrary || "websocket";
      this.transportOptions = options.transportOptions || {};
      this._sendFunc = function(msg) {
        that.sendEncodedMessage(msg);
      };
      if (typeof options.groovyCompatibility === "undefined") {
        this.groovyCompatibility = true;
      } else {
        this.groovyCompatibility = options.groovyCompatibility;
      }
      this.setMaxListeners(0);
      if (options.url) {
        this.connect(options.url);
      }
    }
    Ros.prototype.__proto__ = EventEmitter2.prototype;
    Ros.prototype.connect = function(url) {
      if (this.transportLibrary === "socket.io") {
        this.socket = assign(io(url, { "force new connection": true }), socketAdapter(this));
        this.socket.on("connect", this.socket.onopen);
        this.socket.on("data", this.socket.onmessage);
        this.socket.on("close", this.socket.onclose);
        this.socket.on("error", this.socket.onerror);
      } else if (this.transportLibrary.constructor.name === "RTCPeerConnection") {
        this.socket = assign(this.transportLibrary.createDataChannel(url, this.transportOptions), socketAdapter(this));
      } else if (this.transportLibrary === "websocket") {
        if (!this.socket || this.socket.readyState === WebSocket2.CLOSED) {
          var sock = new WebSocket2(url);
          sock.binaryType = "arraybuffer";
          this.socket = assign(sock, socketAdapter(this));
        }
      } else if (this.transportLibrary === "workersocket") {
        this.socket = assign(new WorkerSocket(url), socketAdapter(this));
      } else {
        throw "Unknown transportLibrary: " + this.transportLibrary.toString();
      }
    };
    Ros.prototype.close = function() {
      if (this.socket) {
        this.socket.close();
      }
    };
    Ros.prototype.authenticate = function(mac, client, dest, rand, t, level, end) {
      var auth = {
        op: "auth",
        mac,
        client,
        dest,
        rand,
        t,
        level,
        end
      };
      this.callOnConnection(auth);
    };
    Ros.prototype.sendEncodedMessage = function(messageEncoded) {
      var emitter = null;
      var that = this;
      if (this.transportLibrary === "socket.io") {
        emitter = function(msg) {
          that.socket.emit("operation", msg);
        };
      } else {
        emitter = function(msg) {
          that.socket.send(msg);
        };
      }
      if (!this.isConnected) {
        that.once("connection", function() {
          emitter(messageEncoded);
        });
      } else {
        emitter(messageEncoded);
      }
    };
    Ros.prototype.callOnConnection = function(message) {
      if (this.transportOptions.encoder) {
        this.transportOptions.encoder(message, this._sendFunc);
      } else {
        this._sendFunc(JSON.stringify(message));
      }
    };
    Ros.prototype.setStatusLevel = function(level, id) {
      var levelMsg = {
        op: "set_level",
        level,
        id
      };
      this.callOnConnection(levelMsg);
    };
    Ros.prototype.getActionServers = function(callback, failedCallback) {
      var getActionServers = new Service({
        ros: this,
        name: "/rosapi/action_servers",
        serviceType: "rosapi/GetActionServers"
      });
      var request = new ServiceRequest({});
      if (typeof failedCallback === "function") {
        getActionServers.callService(
          request,
          function(result) {
            callback(result.action_servers);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        getActionServers.callService(request, function(result) {
          callback(result.action_servers);
        });
      }
    };
    Ros.prototype.getTopics = function(callback, failedCallback) {
      var topicsClient = new Service({
        ros: this,
        name: "/rosapi/topics",
        serviceType: "rosapi/Topics"
      });
      var request = new ServiceRequest();
      if (typeof failedCallback === "function") {
        topicsClient.callService(
          request,
          function(result) {
            callback(result);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        topicsClient.callService(request, function(result) {
          callback(result);
        });
      }
    };
    Ros.prototype.getTopicsForType = function(topicType, callback, failedCallback) {
      var topicsForTypeClient = new Service({
        ros: this,
        name: "/rosapi/topics_for_type",
        serviceType: "rosapi/TopicsForType"
      });
      var request = new ServiceRequest({
        type: topicType
      });
      if (typeof failedCallback === "function") {
        topicsForTypeClient.callService(
          request,
          function(result) {
            callback(result.topics);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        topicsForTypeClient.callService(request, function(result) {
          callback(result.topics);
        });
      }
    };
    Ros.prototype.getServices = function(callback, failedCallback) {
      var servicesClient = new Service({
        ros: this,
        name: "/rosapi/services",
        serviceType: "rosapi/Services"
      });
      var request = new ServiceRequest();
      if (typeof failedCallback === "function") {
        servicesClient.callService(
          request,
          function(result) {
            callback(result.services);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        servicesClient.callService(request, function(result) {
          callback(result.services);
        });
      }
    };
    Ros.prototype.getServicesForType = function(serviceType, callback, failedCallback) {
      var servicesForTypeClient = new Service({
        ros: this,
        name: "/rosapi/services_for_type",
        serviceType: "rosapi/ServicesForType"
      });
      var request = new ServiceRequest({
        type: serviceType
      });
      if (typeof failedCallback === "function") {
        servicesForTypeClient.callService(
          request,
          function(result) {
            callback(result.services);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        servicesForTypeClient.callService(request, function(result) {
          callback(result.services);
        });
      }
    };
    Ros.prototype.getServiceRequestDetails = function(type, callback, failedCallback) {
      var serviceTypeClient = new Service({
        ros: this,
        name: "/rosapi/service_request_details",
        serviceType: "rosapi/ServiceRequestDetails"
      });
      var request = new ServiceRequest({
        type
      });
      if (typeof failedCallback === "function") {
        serviceTypeClient.callService(
          request,
          function(result) {
            callback(result);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        serviceTypeClient.callService(request, function(result) {
          callback(result);
        });
      }
    };
    Ros.prototype.getServiceResponseDetails = function(type, callback, failedCallback) {
      var serviceTypeClient = new Service({
        ros: this,
        name: "/rosapi/service_response_details",
        serviceType: "rosapi/ServiceResponseDetails"
      });
      var request = new ServiceRequest({
        type
      });
      if (typeof failedCallback === "function") {
        serviceTypeClient.callService(
          request,
          function(result) {
            callback(result);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        serviceTypeClient.callService(request, function(result) {
          callback(result);
        });
      }
    };
    Ros.prototype.getNodes = function(callback, failedCallback) {
      var nodesClient = new Service({
        ros: this,
        name: "/rosapi/nodes",
        serviceType: "rosapi/Nodes"
      });
      var request = new ServiceRequest();
      if (typeof failedCallback === "function") {
        nodesClient.callService(
          request,
          function(result) {
            callback(result.nodes);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        nodesClient.callService(request, function(result) {
          callback(result.nodes);
        });
      }
    };
    Ros.prototype.getNodeDetails = function(node, callback, failedCallback) {
      var nodesClient = new Service({
        ros: this,
        name: "/rosapi/node_details",
        serviceType: "rosapi/NodeDetails"
      });
      var request = new ServiceRequest({
        node
      });
      if (typeof failedCallback === "function") {
        nodesClient.callService(
          request,
          function(result) {
            callback(result.subscribing, result.publishing, result.services);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        nodesClient.callService(request, function(result) {
          callback(result);
        });
      }
    };
    Ros.prototype.getParams = function(callback, failedCallback) {
      var paramsClient = new Service({
        ros: this,
        name: "/rosapi/get_param_names",
        serviceType: "rosapi/GetParamNames"
      });
      var request = new ServiceRequest();
      if (typeof failedCallback === "function") {
        paramsClient.callService(
          request,
          function(result) {
            callback(result.names);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        paramsClient.callService(request, function(result) {
          callback(result.names);
        });
      }
    };
    Ros.prototype.getTopicType = function(topic, callback, failedCallback) {
      var topicTypeClient = new Service({
        ros: this,
        name: "/rosapi/topic_type",
        serviceType: "rosapi/TopicType"
      });
      var request = new ServiceRequest({
        topic
      });
      if (typeof failedCallback === "function") {
        topicTypeClient.callService(
          request,
          function(result) {
            callback(result.type);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        topicTypeClient.callService(request, function(result) {
          callback(result.type);
        });
      }
    };
    Ros.prototype.getServiceType = function(service, callback, failedCallback) {
      var serviceTypeClient = new Service({
        ros: this,
        name: "/rosapi/service_type",
        serviceType: "rosapi/ServiceType"
      });
      var request = new ServiceRequest({
        service
      });
      if (typeof failedCallback === "function") {
        serviceTypeClient.callService(
          request,
          function(result) {
            callback(result.type);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        serviceTypeClient.callService(request, function(result) {
          callback(result.type);
        });
      }
    };
    Ros.prototype.getMessageDetails = function(message, callback, failedCallback) {
      var messageDetailClient = new Service({
        ros: this,
        name: "/rosapi/message_details",
        serviceType: "rosapi/MessageDetails"
      });
      var request = new ServiceRequest({
        type: message
      });
      if (typeof failedCallback === "function") {
        messageDetailClient.callService(
          request,
          function(result) {
            callback(result.typedefs);
          },
          function(message2) {
            failedCallback(message2);
          }
        );
      } else {
        messageDetailClient.callService(request, function(result) {
          callback(result.typedefs);
        });
      }
    };
    Ros.prototype.decodeTypeDefs = function(defs) {
      var that = this;
      var decodeTypeDefsRec = function(theType, hints) {
        var typeDefDict = {};
        for (var i = 0; i < theType.fieldnames.length; i++) {
          var arrayLen = theType.fieldarraylen[i];
          var fieldName = theType.fieldnames[i];
          var fieldType = theType.fieldtypes[i];
          if (fieldType.indexOf("/") === -1) {
            if (arrayLen === -1) {
              typeDefDict[fieldName] = fieldType;
            } else {
              typeDefDict[fieldName] = [fieldType];
            }
          } else {
            var sub = false;
            for (var j = 0; j < hints.length; j++) {
              if (hints[j].type.toString() === fieldType.toString()) {
                sub = hints[j];
                break;
              }
            }
            if (sub) {
              var subResult = decodeTypeDefsRec(sub, hints);
              if (arrayLen === -1) {
                typeDefDict[fieldName] = subResult;
              } else {
                typeDefDict[fieldName] = [subResult];
              }
            } else {
              that.emit("error", "Cannot find " + fieldType + " in decodeTypeDefs");
            }
          }
        }
        return typeDefDict;
      };
      return decodeTypeDefsRec(defs[0], defs);
    };
    Ros.prototype.getTopicsAndRawTypes = function(callback, failedCallback) {
      var topicsAndRawTypesClient = new Service({
        ros: this,
        name: "/rosapi/topics_and_raw_types",
        serviceType: "rosapi/TopicsAndRawTypes"
      });
      var request = new ServiceRequest();
      if (typeof failedCallback === "function") {
        topicsAndRawTypesClient.callService(
          request,
          function(result) {
            callback(result);
          },
          function(message) {
            failedCallback(message);
          }
        );
      } else {
        topicsAndRawTypesClient.callService(request, function(result) {
          callback(result);
        });
      }
    };
    module.exports = Ros;
  }
});

// node_modules/roslib/src/core/Message.js
var require_Message = __commonJS({
  "node_modules/roslib/src/core/Message.js"(exports, module) {
    var assign = require_object_assign();
    function Message(values) {
      assign(this, values);
    }
    module.exports = Message;
  }
});

// node_modules/roslib/src/core/Topic.js
var require_Topic = __commonJS({
  "node_modules/roslib/src/core/Topic.js"(exports, module) {
    var EventEmitter2 = require_eventemitter2().EventEmitter2;
    var Message = require_Message();
    function Topic(options) {
      options = options || {};
      this.ros = options.ros;
      this.name = options.name;
      this.messageType = options.messageType;
      this.isAdvertised = false;
      this.compression = options.compression || "none";
      this.throttle_rate = options.throttle_rate || 0;
      this.latch = options.latch || false;
      this.queue_size = options.queue_size || 100;
      this.queue_length = options.queue_length || 0;
      this.reconnect_on_close = options.reconnect_on_close !== void 0 ? options.reconnect_on_close : true;
      if (this.compression && this.compression !== "png" && this.compression !== "cbor" && this.compression !== "cbor-raw" && this.compression !== "none") {
        this.emit("warning", this.compression + " compression is not supported. No compression will be used.");
        this.compression = "none";
      }
      if (this.throttle_rate < 0) {
        this.emit("warning", this.throttle_rate + " is not allowed. Set to 0");
        this.throttle_rate = 0;
      }
      var that = this;
      if (this.reconnect_on_close) {
        this.callForSubscribeAndAdvertise = function(message) {
          that.ros.callOnConnection(message);
          that.waitForReconnect = false;
          that.reconnectFunc = function() {
            if (!that.waitForReconnect) {
              that.waitForReconnect = true;
              that.ros.callOnConnection(message);
              that.ros.once("connection", function() {
                that.waitForReconnect = false;
              });
            }
          };
          that.ros.on("close", that.reconnectFunc);
        };
      } else {
        this.callForSubscribeAndAdvertise = this.ros.callOnConnection;
      }
      this._messageCallback = function(data) {
        that.emit("message", new Message(data));
      };
    }
    Topic.prototype.__proto__ = EventEmitter2.prototype;
    Topic.prototype.subscribe = function(callback) {
      if (typeof callback === "function") {
        this.on("message", callback);
      }
      if (this.subscribeId) {
        return;
      }
      this.ros.on(this.name, this._messageCallback);
      this.subscribeId = "subscribe:" + this.name + ":" + ++this.ros.idCounter;
      this.callForSubscribeAndAdvertise({
        op: "subscribe",
        id: this.subscribeId,
        type: this.messageType,
        topic: this.name,
        compression: this.compression,
        throttle_rate: this.throttle_rate,
        queue_length: this.queue_length
      });
    };
    Topic.prototype.unsubscribe = function(callback) {
      if (callback) {
        this.off("message", callback);
        if (this.listeners("message").length) {
          return;
        }
      }
      if (!this.subscribeId) {
        return;
      }
      this.ros.off(this.name, this._messageCallback);
      if (this.reconnect_on_close) {
        this.ros.off("close", this.reconnectFunc);
      }
      this.emit("unsubscribe");
      this.ros.callOnConnection({
        op: "unsubscribe",
        id: this.subscribeId,
        topic: this.name
      });
      this.subscribeId = null;
    };
    Topic.prototype.advertise = function() {
      if (this.isAdvertised) {
        return;
      }
      this.advertiseId = "advertise:" + this.name + ":" + ++this.ros.idCounter;
      this.callForSubscribeAndAdvertise({
        op: "advertise",
        id: this.advertiseId,
        type: this.messageType,
        topic: this.name,
        latch: this.latch,
        queue_size: this.queue_size
      });
      this.isAdvertised = true;
      if (!this.reconnect_on_close) {
        var that = this;
        this.ros.on("close", function() {
          that.isAdvertised = false;
        });
      }
    };
    Topic.prototype.unadvertise = function() {
      if (!this.isAdvertised) {
        return;
      }
      if (this.reconnect_on_close) {
        this.ros.off("close", this.reconnectFunc);
      }
      this.emit("unadvertise");
      this.ros.callOnConnection({
        op: "unadvertise",
        id: this.advertiseId,
        topic: this.name
      });
      this.isAdvertised = false;
    };
    Topic.prototype.publish = function(message) {
      if (!this.isAdvertised) {
        this.advertise();
      }
      this.ros.idCounter++;
      var call = {
        op: "publish",
        id: "publish:" + this.name + ":" + this.ros.idCounter,
        topic: this.name,
        msg: message,
        latch: this.latch
      };
      this.ros.callOnConnection(call);
    };
    module.exports = Topic;
  }
});

// node_modules/roslib/src/core/Param.js
var require_Param = __commonJS({
  "node_modules/roslib/src/core/Param.js"(exports, module) {
    var Service = require_Service();
    var ServiceRequest = require_ServiceRequest();
    function Param(options) {
      options = options || {};
      this.ros = options.ros;
      this.name = options.name;
    }
    Param.prototype.get = function(callback, failedCallback) {
      var paramClient = new Service({
        ros: this.ros,
        name: "/rosapi/get_param",
        serviceType: "rosapi/GetParam"
      });
      var request = new ServiceRequest({
        name: this.name
      });
      paramClient.callService(request, function(result) {
        var value = JSON.parse(result.value);
        callback(value);
      }, failedCallback);
    };
    Param.prototype.set = function(value, callback, failedCallback) {
      var paramClient = new Service({
        ros: this.ros,
        name: "/rosapi/set_param",
        serviceType: "rosapi/SetParam"
      });
      var request = new ServiceRequest({
        name: this.name,
        value: JSON.stringify(value)
      });
      paramClient.callService(request, callback, failedCallback);
    };
    Param.prototype.delete = function(callback, failedCallback) {
      var paramClient = new Service({
        ros: this.ros,
        name: "/rosapi/delete_param",
        serviceType: "rosapi/DeleteParam"
      });
      var request = new ServiceRequest({
        name: this.name
      });
      paramClient.callService(request, callback, failedCallback);
    };
    module.exports = Param;
  }
});

// node_modules/roslib/src/core/index.js
var require_core = __commonJS({
  "node_modules/roslib/src/core/index.js"(exports, module) {
    var mixin = require_mixin();
    var core = module.exports = {
      Ros: require_Ros(),
      Topic: require_Topic(),
      Message: require_Message(),
      Param: require_Param(),
      Service: require_Service(),
      ServiceRequest: require_ServiceRequest(),
      ServiceResponse: require_ServiceResponse()
    };
    mixin(core.Ros, ["Param", "Service", "Topic"], core);
  }
});

// node_modules/roslib/src/actionlib/ActionClient.js
var require_ActionClient = __commonJS({
  "node_modules/roslib/src/actionlib/ActionClient.js"(exports, module) {
    var Topic = require_Topic();
    var Message = require_Message();
    var EventEmitter2 = require_eventemitter2().EventEmitter2;
    function ActionClient(options) {
      var that = this;
      options = options || {};
      this.ros = options.ros;
      this.serverName = options.serverName;
      this.actionName = options.actionName;
      this.timeout = options.timeout;
      this.omitFeedback = options.omitFeedback;
      this.omitStatus = options.omitStatus;
      this.omitResult = options.omitResult;
      this.goals = {};
      var receivedStatus = false;
      this.feedbackListener = new Topic({
        ros: this.ros,
        name: this.serverName + "/feedback",
        messageType: this.actionName + "Feedback"
      });
      this.statusListener = new Topic({
        ros: this.ros,
        name: this.serverName + "/status",
        messageType: "actionlib_msgs/GoalStatusArray"
      });
      this.resultListener = new Topic({
        ros: this.ros,
        name: this.serverName + "/result",
        messageType: this.actionName + "Result"
      });
      this.goalTopic = new Topic({
        ros: this.ros,
        name: this.serverName + "/goal",
        messageType: this.actionName + "Goal"
      });
      this.cancelTopic = new Topic({
        ros: this.ros,
        name: this.serverName + "/cancel",
        messageType: "actionlib_msgs/GoalID"
      });
      this.goalTopic.advertise();
      this.cancelTopic.advertise();
      if (!this.omitStatus) {
        this.statusListener.subscribe(function(statusMessage) {
          receivedStatus = true;
          statusMessage.status_list.forEach(function(status) {
            var goal = that.goals[status.goal_id.id];
            if (goal) {
              goal.emit("status", status);
            }
          });
        });
      }
      if (!this.omitFeedback) {
        this.feedbackListener.subscribe(function(feedbackMessage) {
          var goal = that.goals[feedbackMessage.status.goal_id.id];
          if (goal) {
            goal.emit("status", feedbackMessage.status);
            goal.emit("feedback", feedbackMessage.feedback);
          }
        });
      }
      if (!this.omitResult) {
        this.resultListener.subscribe(function(resultMessage) {
          var goal = that.goals[resultMessage.status.goal_id.id];
          if (goal) {
            goal.emit("status", resultMessage.status);
            goal.emit("result", resultMessage.result);
          }
        });
      }
      if (this.timeout) {
        setTimeout(function() {
          if (!receivedStatus) {
            that.emit("timeout");
          }
        }, this.timeout);
      }
    }
    ActionClient.prototype.__proto__ = EventEmitter2.prototype;
    ActionClient.prototype.cancel = function() {
      var cancelMessage = new Message();
      this.cancelTopic.publish(cancelMessage);
    };
    ActionClient.prototype.dispose = function() {
      this.goalTopic.unadvertise();
      this.cancelTopic.unadvertise();
      if (!this.omitStatus) {
        this.statusListener.unsubscribe();
      }
      if (!this.omitFeedback) {
        this.feedbackListener.unsubscribe();
      }
      if (!this.omitResult) {
        this.resultListener.unsubscribe();
      }
    };
    module.exports = ActionClient;
  }
});

// node_modules/roslib/src/actionlib/ActionListener.js
var require_ActionListener = __commonJS({
  "node_modules/roslib/src/actionlib/ActionListener.js"(exports, module) {
    var Topic = require_Topic();
    var Message = require_Message();
    var EventEmitter2 = require_eventemitter2().EventEmitter2;
    function ActionListener(options) {
      var that = this;
      options = options || {};
      this.ros = options.ros;
      this.serverName = options.serverName;
      this.actionName = options.actionName;
      var goalListener = new Topic({
        ros: this.ros,
        name: this.serverName + "/goal",
        messageType: this.actionName + "Goal"
      });
      var feedbackListener = new Topic({
        ros: this.ros,
        name: this.serverName + "/feedback",
        messageType: this.actionName + "Feedback"
      });
      var statusListener = new Topic({
        ros: this.ros,
        name: this.serverName + "/status",
        messageType: "actionlib_msgs/GoalStatusArray"
      });
      var resultListener = new Topic({
        ros: this.ros,
        name: this.serverName + "/result",
        messageType: this.actionName + "Result"
      });
      goalListener.subscribe(function(goalMessage) {
        that.emit("goal", goalMessage);
      });
      statusListener.subscribe(function(statusMessage) {
        statusMessage.status_list.forEach(function(status) {
          that.emit("status", status);
        });
      });
      feedbackListener.subscribe(function(feedbackMessage) {
        that.emit("status", feedbackMessage.status);
        that.emit("feedback", feedbackMessage.feedback);
      });
      resultListener.subscribe(function(resultMessage) {
        that.emit("status", resultMessage.status);
        that.emit("result", resultMessage.result);
      });
    }
    ActionListener.prototype.__proto__ = EventEmitter2.prototype;
    module.exports = ActionListener;
  }
});

// node_modules/roslib/src/actionlib/Goal.js
var require_Goal = __commonJS({
  "node_modules/roslib/src/actionlib/Goal.js"(exports, module) {
    var Message = require_Message();
    var EventEmitter2 = require_eventemitter2().EventEmitter2;
    function Goal(options) {
      var that = this;
      this.actionClient = options.actionClient;
      this.goalMessage = options.goalMessage;
      this.isFinished = false;
      var date = /* @__PURE__ */ new Date();
      this.goalID = "goal_" + Math.random() + "_" + date.getTime();
      this.goalMessage = new Message({
        goal_id: {
          stamp: {
            secs: 0,
            nsecs: 0
          },
          id: this.goalID
        },
        goal: this.goalMessage
      });
      this.on("status", function(status) {
        that.status = status;
      });
      this.on("result", function(result) {
        that.isFinished = true;
        that.result = result;
      });
      this.on("feedback", function(feedback) {
        that.feedback = feedback;
      });
      this.actionClient.goals[this.goalID] = this;
    }
    Goal.prototype.__proto__ = EventEmitter2.prototype;
    Goal.prototype.send = function(timeout) {
      var that = this;
      that.actionClient.goalTopic.publish(that.goalMessage);
      if (timeout) {
        setTimeout(function() {
          if (!that.isFinished) {
            that.emit("timeout");
          }
        }, timeout);
      }
    };
    Goal.prototype.cancel = function() {
      var cancelMessage = new Message({
        id: this.goalID
      });
      this.actionClient.cancelTopic.publish(cancelMessage);
    };
    module.exports = Goal;
  }
});

// node_modules/roslib/src/actionlib/SimpleActionServer.js
var require_SimpleActionServer = __commonJS({
  "node_modules/roslib/src/actionlib/SimpleActionServer.js"(exports, module) {
    var Topic = require_Topic();
    var Message = require_Message();
    var EventEmitter2 = require_eventemitter2().EventEmitter2;
    function SimpleActionServer(options) {
      var that = this;
      options = options || {};
      this.ros = options.ros;
      this.serverName = options.serverName;
      this.actionName = options.actionName;
      this.feedbackPublisher = new Topic({
        ros: this.ros,
        name: this.serverName + "/feedback",
        messageType: this.actionName + "Feedback"
      });
      this.feedbackPublisher.advertise();
      var statusPublisher = new Topic({
        ros: this.ros,
        name: this.serverName + "/status",
        messageType: "actionlib_msgs/GoalStatusArray"
      });
      statusPublisher.advertise();
      this.resultPublisher = new Topic({
        ros: this.ros,
        name: this.serverName + "/result",
        messageType: this.actionName + "Result"
      });
      this.resultPublisher.advertise();
      var goalListener = new Topic({
        ros: this.ros,
        name: this.serverName + "/goal",
        messageType: this.actionName + "Goal"
      });
      var cancelListener = new Topic({
        ros: this.ros,
        name: this.serverName + "/cancel",
        messageType: "actionlib_msgs/GoalID"
      });
      this.statusMessage = new Message({
        header: {
          stamp: { secs: 0, nsecs: 100 },
          frame_id: ""
        },
        status_list: []
      });
      this.currentGoal = null;
      this.nextGoal = null;
      goalListener.subscribe(function(goalMessage) {
        if (that.currentGoal) {
          that.nextGoal = goalMessage;
          that.emit("cancel");
        } else {
          that.statusMessage.status_list = [{ goal_id: goalMessage.goal_id, status: 1 }];
          that.currentGoal = goalMessage;
          that.emit("goal", goalMessage.goal);
        }
      });
      var isEarlier = function(t1, t2) {
        if (t1.secs > t2.secs) {
          return false;
        } else if (t1.secs < t2.secs) {
          return true;
        } else if (t1.nsecs < t2.nsecs) {
          return true;
        } else {
          return false;
        }
      };
      cancelListener.subscribe(function(cancelMessage) {
        if (cancelMessage.stamp.secs === 0 && cancelMessage.stamp.secs === 0 && cancelMessage.id === "") {
          that.nextGoal = null;
          if (that.currentGoal) {
            that.emit("cancel");
          }
        } else {
          if (that.currentGoal && cancelMessage.id === that.currentGoal.goal_id.id) {
            that.emit("cancel");
          } else if (that.nextGoal && cancelMessage.id === that.nextGoal.goal_id.id) {
            that.nextGoal = null;
          }
          if (that.nextGoal && isEarlier(
            that.nextGoal.goal_id.stamp,
            cancelMessage.stamp
          )) {
            that.nextGoal = null;
          }
          if (that.currentGoal && isEarlier(
            that.currentGoal.goal_id.stamp,
            cancelMessage.stamp
          )) {
            that.emit("cancel");
          }
        }
      });
      var statusInterval = setInterval(function() {
        var currentTime = /* @__PURE__ */ new Date();
        var secs = Math.floor(currentTime.getTime() / 1e3);
        var nsecs = Math.round(1e9 * (currentTime.getTime() / 1e3 - secs));
        that.statusMessage.header.stamp.secs = secs;
        that.statusMessage.header.stamp.nsecs = nsecs;
        statusPublisher.publish(that.statusMessage);
      }, 500);
    }
    SimpleActionServer.prototype.__proto__ = EventEmitter2.prototype;
    SimpleActionServer.prototype.setSucceeded = function(result) {
      var resultMessage = new Message({
        status: { goal_id: this.currentGoal.goal_id, status: 3 },
        result
      });
      this.resultPublisher.publish(resultMessage);
      this.statusMessage.status_list = [];
      if (this.nextGoal) {
        this.currentGoal = this.nextGoal;
        this.nextGoal = null;
        this.emit("goal", this.currentGoal.goal);
      } else {
        this.currentGoal = null;
      }
    };
    SimpleActionServer.prototype.setAborted = function(result) {
      var resultMessage = new Message({
        status: { goal_id: this.currentGoal.goal_id, status: 4 },
        result
      });
      this.resultPublisher.publish(resultMessage);
      this.statusMessage.status_list = [];
      if (this.nextGoal) {
        this.currentGoal = this.nextGoal;
        this.nextGoal = null;
        this.emit("goal", this.currentGoal.goal);
      } else {
        this.currentGoal = null;
      }
    };
    SimpleActionServer.prototype.sendFeedback = function(feedback) {
      var feedbackMessage = new Message({
        status: { goal_id: this.currentGoal.goal_id, status: 1 },
        feedback
      });
      this.feedbackPublisher.publish(feedbackMessage);
    };
    SimpleActionServer.prototype.setPreempted = function() {
      this.statusMessage.status_list = [];
      var resultMessage = new Message({
        status: { goal_id: this.currentGoal.goal_id, status: 2 }
      });
      this.resultPublisher.publish(resultMessage);
      if (this.nextGoal) {
        this.currentGoal = this.nextGoal;
        this.nextGoal = null;
        this.emit("goal", this.currentGoal.goal);
      } else {
        this.currentGoal = null;
      }
    };
    module.exports = SimpleActionServer;
  }
});

// node_modules/roslib/src/actionlib/index.js
var require_actionlib = __commonJS({
  "node_modules/roslib/src/actionlib/index.js"(exports, module) {
    var Ros = require_Ros();
    var mixin = require_mixin();
    var action = module.exports = {
      ActionClient: require_ActionClient(),
      ActionListener: require_ActionListener(),
      Goal: require_Goal(),
      SimpleActionServer: require_SimpleActionServer()
    };
    mixin(Ros, ["ActionClient", "SimpleActionServer"], action);
  }
});

// node_modules/roslib/src/math/Vector3.js
var require_Vector3 = __commonJS({
  "node_modules/roslib/src/math/Vector3.js"(exports, module) {
    function Vector3(options) {
      options = options || {};
      this.x = options.x || 0;
      this.y = options.y || 0;
      this.z = options.z || 0;
    }
    Vector3.prototype.add = function(v) {
      this.x += v.x;
      this.y += v.y;
      this.z += v.z;
    };
    Vector3.prototype.subtract = function(v) {
      this.x -= v.x;
      this.y -= v.y;
      this.z -= v.z;
    };
    Vector3.prototype.multiplyQuaternion = function(q) {
      var ix = q.w * this.x + q.y * this.z - q.z * this.y;
      var iy = q.w * this.y + q.z * this.x - q.x * this.z;
      var iz = q.w * this.z + q.x * this.y - q.y * this.x;
      var iw = -q.x * this.x - q.y * this.y - q.z * this.z;
      this.x = ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y;
      this.y = iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z;
      this.z = iz * q.w + iw * -q.z + ix * -q.y - iy * -q.x;
    };
    Vector3.prototype.clone = function() {
      return new Vector3(this);
    };
    module.exports = Vector3;
  }
});

// node_modules/roslib/src/math/Quaternion.js
var require_Quaternion = __commonJS({
  "node_modules/roslib/src/math/Quaternion.js"(exports, module) {
    function Quaternion(options) {
      options = options || {};
      this.x = options.x || 0;
      this.y = options.y || 0;
      this.z = options.z || 0;
      this.w = typeof options.w === "number" ? options.w : 1;
    }
    Quaternion.prototype.conjugate = function() {
      this.x *= -1;
      this.y *= -1;
      this.z *= -1;
    };
    Quaternion.prototype.norm = function() {
      return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w);
    };
    Quaternion.prototype.normalize = function() {
      var l = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w);
      if (l === 0) {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.w = 1;
      } else {
        l = 1 / l;
        this.x = this.x * l;
        this.y = this.y * l;
        this.z = this.z * l;
        this.w = this.w * l;
      }
    };
    Quaternion.prototype.invert = function() {
      this.conjugate();
      this.normalize();
    };
    Quaternion.prototype.multiply = function(q) {
      var newX = this.x * q.w + this.y * q.z - this.z * q.y + this.w * q.x;
      var newY = -this.x * q.z + this.y * q.w + this.z * q.x + this.w * q.y;
      var newZ = this.x * q.y - this.y * q.x + this.z * q.w + this.w * q.z;
      var newW = -this.x * q.x - this.y * q.y - this.z * q.z + this.w * q.w;
      this.x = newX;
      this.y = newY;
      this.z = newZ;
      this.w = newW;
    };
    Quaternion.prototype.clone = function() {
      return new Quaternion(this);
    };
    module.exports = Quaternion;
  }
});

// node_modules/roslib/src/math/Pose.js
var require_Pose = __commonJS({
  "node_modules/roslib/src/math/Pose.js"(exports, module) {
    var Vector3 = require_Vector3();
    var Quaternion = require_Quaternion();
    function Pose(options) {
      options = options || {};
      this.position = new Vector3(options.position);
      this.orientation = new Quaternion(options.orientation);
    }
    Pose.prototype.applyTransform = function(tf) {
      this.position.multiplyQuaternion(tf.rotation);
      this.position.add(tf.translation);
      var tmp = tf.rotation.clone();
      tmp.multiply(this.orientation);
      this.orientation = tmp;
    };
    Pose.prototype.clone = function() {
      return new Pose(this);
    };
    Pose.prototype.multiply = function(pose) {
      var p = pose.clone();
      p.applyTransform({ rotation: this.orientation, translation: this.position });
      return p;
    };
    Pose.prototype.getInverse = function() {
      var inverse = this.clone();
      inverse.orientation.invert();
      inverse.position.multiplyQuaternion(inverse.orientation);
      inverse.position.x *= -1;
      inverse.position.y *= -1;
      inverse.position.z *= -1;
      return inverse;
    };
    module.exports = Pose;
  }
});

// node_modules/roslib/src/math/Transform.js
var require_Transform = __commonJS({
  "node_modules/roslib/src/math/Transform.js"(exports, module) {
    var Vector3 = require_Vector3();
    var Quaternion = require_Quaternion();
    function Transform(options) {
      options = options || {};
      this.translation = new Vector3(options.translation);
      this.rotation = new Quaternion(options.rotation);
    }
    Transform.prototype.clone = function() {
      return new Transform(this);
    };
    module.exports = Transform;
  }
});

// node_modules/roslib/src/math/index.js
var require_math = __commonJS({
  "node_modules/roslib/src/math/index.js"(exports, module) {
    module.exports = {
      Pose: require_Pose(),
      Quaternion: require_Quaternion(),
      Transform: require_Transform(),
      Vector3: require_Vector3()
    };
  }
});

// node_modules/roslib/src/tf/TFClient.js
var require_TFClient = __commonJS({
  "node_modules/roslib/src/tf/TFClient.js"(exports, module) {
    var ActionClient = require_ActionClient();
    var Goal = require_Goal();
    var Service = require_Service();
    var ServiceRequest = require_ServiceRequest();
    var Topic = require_Topic();
    var Transform = require_Transform();
    function TFClient(options) {
      options = options || {};
      this.ros = options.ros;
      this.fixedFrame = options.fixedFrame || "base_link";
      this.angularThres = options.angularThres || 2;
      this.transThres = options.transThres || 0.01;
      this.rate = options.rate || 10;
      this.updateDelay = options.updateDelay || 50;
      var seconds = options.topicTimeout || 2;
      var secs = Math.floor(seconds);
      var nsecs = Math.floor((seconds - secs) * 1e9);
      this.topicTimeout = {
        secs,
        nsecs
      };
      this.serverName = options.serverName || "/tf2_web_republisher";
      this.repubServiceName = options.repubServiceName || "/republish_tfs";
      this.currentGoal = false;
      this.currentTopic = false;
      this.frameInfos = {};
      this.republisherUpdateRequested = false;
      this._subscribeCB = null;
      this._isDisposed = false;
      this.actionClient = new ActionClient({
        ros: options.ros,
        serverName: this.serverName,
        actionName: "tf2_web_republisher/TFSubscriptionAction",
        omitStatus: true,
        omitResult: true
      });
      this.serviceClient = new Service({
        ros: options.ros,
        name: this.repubServiceName,
        serviceType: "tf2_web_republisher/RepublishTFs"
      });
    }
    TFClient.prototype.processTFArray = function(tf) {
      var that = this;
      tf.transforms.forEach(function(transform) {
        var frameID = transform.child_frame_id;
        if (frameID[0] === "/") {
          frameID = frameID.substring(1);
        }
        var info = this.frameInfos[frameID];
        if (info) {
          info.transform = new Transform({
            translation: transform.transform.translation,
            rotation: transform.transform.rotation
          });
          info.cbs.forEach(function(cb) {
            cb(info.transform);
          });
        }
      }, this);
    };
    TFClient.prototype.updateGoal = function() {
      var goalMessage = {
        source_frames: Object.keys(this.frameInfos),
        target_frame: this.fixedFrame,
        angular_thres: this.angularThres,
        trans_thres: this.transThres,
        rate: this.rate
      };
      if (this.ros.groovyCompatibility) {
        if (this.currentGoal) {
          this.currentGoal.cancel();
        }
        this.currentGoal = new Goal({
          actionClient: this.actionClient,
          goalMessage
        });
        this.currentGoal.on("feedback", this.processTFArray.bind(this));
        this.currentGoal.send();
      } else {
        goalMessage.timeout = this.topicTimeout;
        var request = new ServiceRequest(goalMessage);
        this.serviceClient.callService(request, this.processResponse.bind(this));
      }
      this.republisherUpdateRequested = false;
    };
    TFClient.prototype.processResponse = function(response) {
      if (this._isDisposed) {
        return;
      }
      if (this.currentTopic) {
        this.currentTopic.unsubscribe(this._subscribeCB);
      }
      this.currentTopic = new Topic({
        ros: this.ros,
        name: response.topic_name,
        messageType: "tf2_web_republisher/TFArray"
      });
      this._subscribeCB = this.processTFArray.bind(this);
      this.currentTopic.subscribe(this._subscribeCB);
    };
    TFClient.prototype.subscribe = function(frameID, callback) {
      if (frameID[0] === "/") {
        frameID = frameID.substring(1);
      }
      if (!this.frameInfos[frameID]) {
        this.frameInfos[frameID] = {
          cbs: []
        };
        if (!this.republisherUpdateRequested) {
          setTimeout(this.updateGoal.bind(this), this.updateDelay);
          this.republisherUpdateRequested = true;
        }
      } else if (this.frameInfos[frameID].transform) {
        callback(this.frameInfos[frameID].transform);
      }
      this.frameInfos[frameID].cbs.push(callback);
    };
    TFClient.prototype.unsubscribe = function(frameID, callback) {
      if (frameID[0] === "/") {
        frameID = frameID.substring(1);
      }
      var info = this.frameInfos[frameID];
      for (var cbs = info && info.cbs || [], idx = cbs.length; idx--; ) {
        if (cbs[idx] === callback) {
          cbs.splice(idx, 1);
        }
      }
      if (!callback || cbs.length === 0) {
        delete this.frameInfos[frameID];
      }
    };
    TFClient.prototype.dispose = function() {
      this._isDisposed = true;
      this.actionClient.dispose();
      if (this.currentTopic) {
        this.currentTopic.unsubscribe(this._subscribeCB);
      }
    };
    module.exports = TFClient;
  }
});

// node_modules/roslib/src/tf/index.js
var require_tf = __commonJS({
  "node_modules/roslib/src/tf/index.js"(exports, module) {
    var Ros = require_Ros();
    var mixin = require_mixin();
    var tf = module.exports = {
      TFClient: require_TFClient()
    };
    mixin(Ros, ["TFClient"], tf);
  }
});

// node_modules/roslib/src/urdf/UrdfTypes.js
var require_UrdfTypes = __commonJS({
  "node_modules/roslib/src/urdf/UrdfTypes.js"(exports, module) {
    module.exports = {
      URDF_SPHERE: 0,
      URDF_BOX: 1,
      URDF_CYLINDER: 2,
      URDF_MESH: 3
    };
  }
});

// node_modules/roslib/src/urdf/UrdfBox.js
var require_UrdfBox = __commonJS({
  "node_modules/roslib/src/urdf/UrdfBox.js"(exports, module) {
    var Vector3 = require_Vector3();
    var UrdfTypes = require_UrdfTypes();
    function UrdfBox(options) {
      this.dimension = null;
      this.type = UrdfTypes.URDF_BOX;
      var xyz = options.xml.getAttribute("size").split(" ");
      this.dimension = new Vector3({
        x: parseFloat(xyz[0]),
        y: parseFloat(xyz[1]),
        z: parseFloat(xyz[2])
      });
    }
    module.exports = UrdfBox;
  }
});

// node_modules/roslib/src/urdf/UrdfColor.js
var require_UrdfColor = __commonJS({
  "node_modules/roslib/src/urdf/UrdfColor.js"(exports, module) {
    function UrdfColor(options) {
      var rgba = options.xml.getAttribute("rgba").split(" ");
      this.r = parseFloat(rgba[0]);
      this.g = parseFloat(rgba[1]);
      this.b = parseFloat(rgba[2]);
      this.a = parseFloat(rgba[3]);
    }
    module.exports = UrdfColor;
  }
});

// node_modules/roslib/src/urdf/UrdfCylinder.js
var require_UrdfCylinder = __commonJS({
  "node_modules/roslib/src/urdf/UrdfCylinder.js"(exports, module) {
    var UrdfTypes = require_UrdfTypes();
    function UrdfCylinder(options) {
      this.type = UrdfTypes.URDF_CYLINDER;
      this.length = parseFloat(options.xml.getAttribute("length"));
      this.radius = parseFloat(options.xml.getAttribute("radius"));
    }
    module.exports = UrdfCylinder;
  }
});

// node_modules/roslib/src/urdf/UrdfMaterial.js
var require_UrdfMaterial = __commonJS({
  "node_modules/roslib/src/urdf/UrdfMaterial.js"(exports, module) {
    var UrdfColor = require_UrdfColor();
    function UrdfMaterial(options) {
      this.textureFilename = null;
      this.color = null;
      this.name = options.xml.getAttribute("name");
      var textures = options.xml.getElementsByTagName("texture");
      if (textures.length > 0) {
        this.textureFilename = textures[0].getAttribute("filename");
      }
      var colors = options.xml.getElementsByTagName("color");
      if (colors.length > 0) {
        this.color = new UrdfColor({
          xml: colors[0]
        });
      }
    }
    UrdfMaterial.prototype.isLink = function() {
      return this.color === null && this.textureFilename === null;
    };
    var assign = require_object_assign();
    UrdfMaterial.prototype.assign = function(obj) {
      return assign(this, obj);
    };
    module.exports = UrdfMaterial;
  }
});

// node_modules/roslib/src/urdf/UrdfMesh.js
var require_UrdfMesh = __commonJS({
  "node_modules/roslib/src/urdf/UrdfMesh.js"(exports, module) {
    var Vector3 = require_Vector3();
    var UrdfTypes = require_UrdfTypes();
    function UrdfMesh(options) {
      this.scale = null;
      this.type = UrdfTypes.URDF_MESH;
      this.filename = options.xml.getAttribute("filename");
      var scale = options.xml.getAttribute("scale");
      if (scale) {
        var xyz = scale.split(" ");
        this.scale = new Vector3({
          x: parseFloat(xyz[0]),
          y: parseFloat(xyz[1]),
          z: parseFloat(xyz[2])
        });
      }
    }
    module.exports = UrdfMesh;
  }
});

// node_modules/roslib/src/urdf/UrdfSphere.js
var require_UrdfSphere = __commonJS({
  "node_modules/roslib/src/urdf/UrdfSphere.js"(exports, module) {
    var UrdfTypes = require_UrdfTypes();
    function UrdfSphere(options) {
      this.type = UrdfTypes.URDF_SPHERE;
      this.radius = parseFloat(options.xml.getAttribute("radius"));
    }
    module.exports = UrdfSphere;
  }
});

// node_modules/roslib/src/urdf/UrdfVisual.js
var require_UrdfVisual = __commonJS({
  "node_modules/roslib/src/urdf/UrdfVisual.js"(exports, module) {
    var Pose = require_Pose();
    var Vector3 = require_Vector3();
    var Quaternion = require_Quaternion();
    var UrdfCylinder = require_UrdfCylinder();
    var UrdfBox = require_UrdfBox();
    var UrdfMaterial = require_UrdfMaterial();
    var UrdfMesh = require_UrdfMesh();
    var UrdfSphere = require_UrdfSphere();
    function UrdfVisual(options) {
      var xml = options.xml;
      this.origin = null;
      this.geometry = null;
      this.material = null;
      this.name = options.xml.getAttribute("name");
      var origins = xml.getElementsByTagName("origin");
      if (origins.length === 0) {
        this.origin = new Pose();
      } else {
        var xyz = origins[0].getAttribute("xyz");
        var position = new Vector3();
        if (xyz) {
          xyz = xyz.split(" ");
          position = new Vector3({
            x: parseFloat(xyz[0]),
            y: parseFloat(xyz[1]),
            z: parseFloat(xyz[2])
          });
        }
        var rpy = origins[0].getAttribute("rpy");
        var orientation = new Quaternion();
        if (rpy) {
          rpy = rpy.split(" ");
          var roll = parseFloat(rpy[0]);
          var pitch = parseFloat(rpy[1]);
          var yaw = parseFloat(rpy[2]);
          var phi = roll / 2;
          var the = pitch / 2;
          var psi = yaw / 2;
          var x = Math.sin(phi) * Math.cos(the) * Math.cos(psi) - Math.cos(phi) * Math.sin(the) * Math.sin(psi);
          var y = Math.cos(phi) * Math.sin(the) * Math.cos(psi) + Math.sin(phi) * Math.cos(the) * Math.sin(psi);
          var z = Math.cos(phi) * Math.cos(the) * Math.sin(psi) - Math.sin(phi) * Math.sin(the) * Math.cos(psi);
          var w = Math.cos(phi) * Math.cos(the) * Math.cos(psi) + Math.sin(phi) * Math.sin(the) * Math.sin(psi);
          orientation = new Quaternion({
            x,
            y,
            z,
            w
          });
          orientation.normalize();
        }
        this.origin = new Pose({
          position,
          orientation
        });
      }
      var geoms = xml.getElementsByTagName("geometry");
      if (geoms.length > 0) {
        var geom = geoms[0];
        var shape = null;
        for (var i = 0; i < geom.childNodes.length; i++) {
          var node = geom.childNodes[i];
          if (node.nodeType === 1) {
            shape = node;
            break;
          }
        }
        var type = shape.nodeName;
        if (type === "sphere") {
          this.geometry = new UrdfSphere({
            xml: shape
          });
        } else if (type === "box") {
          this.geometry = new UrdfBox({
            xml: shape
          });
        } else if (type === "cylinder") {
          this.geometry = new UrdfCylinder({
            xml: shape
          });
        } else if (type === "mesh") {
          this.geometry = new UrdfMesh({
            xml: shape
          });
        } else {
          console.warn("Unknown geometry type " + type);
        }
      }
      var materials = xml.getElementsByTagName("material");
      if (materials.length > 0) {
        this.material = new UrdfMaterial({
          xml: materials[0]
        });
      }
    }
    module.exports = UrdfVisual;
  }
});

// node_modules/roslib/src/urdf/UrdfLink.js
var require_UrdfLink = __commonJS({
  "node_modules/roslib/src/urdf/UrdfLink.js"(exports, module) {
    var UrdfVisual = require_UrdfVisual();
    function UrdfLink(options) {
      this.name = options.xml.getAttribute("name");
      this.visuals = [];
      var visuals = options.xml.getElementsByTagName("visual");
      for (var i = 0; i < visuals.length; i++) {
        this.visuals.push(new UrdfVisual({
          xml: visuals[i]
        }));
      }
    }
    module.exports = UrdfLink;
  }
});

// node_modules/roslib/src/urdf/UrdfJoint.js
var require_UrdfJoint = __commonJS({
  "node_modules/roslib/src/urdf/UrdfJoint.js"(exports, module) {
    var Pose = require_Pose();
    var Vector3 = require_Vector3();
    var Quaternion = require_Quaternion();
    function UrdfJoint(options) {
      this.name = options.xml.getAttribute("name");
      this.type = options.xml.getAttribute("type");
      var parents = options.xml.getElementsByTagName("parent");
      if (parents.length > 0) {
        this.parent = parents[0].getAttribute("link");
      }
      var children = options.xml.getElementsByTagName("child");
      if (children.length > 0) {
        this.child = children[0].getAttribute("link");
      }
      var limits = options.xml.getElementsByTagName("limit");
      if (limits.length > 0) {
        this.minval = parseFloat(limits[0].getAttribute("lower"));
        this.maxval = parseFloat(limits[0].getAttribute("upper"));
      }
      var origins = options.xml.getElementsByTagName("origin");
      if (origins.length === 0) {
        this.origin = new Pose();
      } else {
        var xyz = origins[0].getAttribute("xyz");
        var position = new Vector3();
        if (xyz) {
          xyz = xyz.split(" ");
          position = new Vector3({
            x: parseFloat(xyz[0]),
            y: parseFloat(xyz[1]),
            z: parseFloat(xyz[2])
          });
        }
        var rpy = origins[0].getAttribute("rpy");
        var orientation = new Quaternion();
        if (rpy) {
          rpy = rpy.split(" ");
          var roll = parseFloat(rpy[0]);
          var pitch = parseFloat(rpy[1]);
          var yaw = parseFloat(rpy[2]);
          var phi = roll / 2;
          var the = pitch / 2;
          var psi = yaw / 2;
          var x = Math.sin(phi) * Math.cos(the) * Math.cos(psi) - Math.cos(phi) * Math.sin(the) * Math.sin(psi);
          var y = Math.cos(phi) * Math.sin(the) * Math.cos(psi) + Math.sin(phi) * Math.cos(the) * Math.sin(psi);
          var z = Math.cos(phi) * Math.cos(the) * Math.sin(psi) - Math.sin(phi) * Math.sin(the) * Math.cos(psi);
          var w = Math.cos(phi) * Math.cos(the) * Math.cos(psi) + Math.sin(phi) * Math.sin(the) * Math.sin(psi);
          orientation = new Quaternion({
            x,
            y,
            z,
            w
          });
          orientation.normalize();
        }
        this.origin = new Pose({
          position,
          orientation
        });
      }
    }
    module.exports = UrdfJoint;
  }
});

// node_modules/roslib/src/util/shim/@xmldom/xmldom.js
var require_xmldom = __commonJS({
  "node_modules/roslib/src/util/shim/@xmldom/xmldom.js"(exports) {
    exports.DOMImplementation = window.DOMImplementation;
    exports.XMLSerializer = window.XMLSerializer;
    exports.DOMParser = window.DOMParser;
  }
});

// node_modules/roslib/src/urdf/UrdfModel.js
var require_UrdfModel = __commonJS({
  "node_modules/roslib/src/urdf/UrdfModel.js"(exports, module) {
    var UrdfMaterial = require_UrdfMaterial();
    var UrdfLink = require_UrdfLink();
    var UrdfJoint = require_UrdfJoint();
    var DOMParser = require_xmldom().DOMParser;
    function UrdfModel(options) {
      options = options || {};
      var xmlDoc = options.xml;
      var string = options.string;
      this.materials = {};
      this.links = {};
      this.joints = {};
      if (string) {
        var parser = new DOMParser();
        xmlDoc = parser.parseFromString(string, "text/xml");
      }
      var robotXml = xmlDoc.documentElement;
      this.name = robotXml.getAttribute("name");
      for (var nodes = robotXml.childNodes, i = 0; i < nodes.length; i++) {
        var node = nodes[i];
        if (node.tagName === "material") {
          var material = new UrdfMaterial({
            xml: node
          });
          if (this.materials[material.name] !== void 0) {
            if (this.materials[material.name].isLink()) {
              this.materials[material.name].assign(material);
            } else {
              console.warn("Material " + material.name + "is not unique.");
            }
          } else {
            this.materials[material.name] = material;
          }
        } else if (node.tagName === "link") {
          var link = new UrdfLink({
            xml: node
          });
          if (this.links[link.name] !== void 0) {
            console.warn("Link " + link.name + " is not unique.");
          } else {
            for (var j = 0; j < link.visuals.length; j++) {
              var mat = link.visuals[j].material;
              if (mat !== null && mat.name) {
                if (this.materials[mat.name] !== void 0) {
                  link.visuals[j].material = this.materials[mat.name];
                } else {
                  this.materials[mat.name] = mat;
                }
              }
            }
            this.links[link.name] = link;
          }
        } else if (node.tagName === "joint") {
          var joint = new UrdfJoint({
            xml: node
          });
          this.joints[joint.name] = joint;
        }
      }
    }
    module.exports = UrdfModel;
  }
});

// node_modules/roslib/src/urdf/index.js
var require_urdf = __commonJS({
  "node_modules/roslib/src/urdf/index.js"(exports, module) {
    module.exports = require_object_assign()({
      UrdfBox: require_UrdfBox(),
      UrdfColor: require_UrdfColor(),
      UrdfCylinder: require_UrdfCylinder(),
      UrdfLink: require_UrdfLink(),
      UrdfMaterial: require_UrdfMaterial(),
      UrdfMesh: require_UrdfMesh(),
      UrdfModel: require_UrdfModel(),
      UrdfSphere: require_UrdfSphere(),
      UrdfVisual: require_UrdfVisual()
    }, require_UrdfTypes());
  }
});

// node_modules/roslib/src/RosLib.js
var require_RosLib = __commonJS({
  "node_modules/roslib/src/RosLib.js"(exports, module) {
    var ROSLIB = exports.ROSLIB || {
      /**
       * @default
       * @description Library version
       */
      REVISION: "1.4.1"
    };
    var assign = require_object_assign();
    assign(ROSLIB, require_core());
    assign(ROSLIB, require_actionlib());
    assign(ROSLIB, require_math());
    assign(ROSLIB, require_tf());
    assign(ROSLIB, require_urdf());
    module.exports = ROSLIB;
  }
});
export default require_RosLib();
/*! Bundled license information:

object-assign/index.js:
  (*
  object-assign
  (c) Sindre Sorhus
  @license MIT
  *)

eventemitter2/lib/eventemitter2.js:
  (*!
   * EventEmitter2
   * https://github.com/hij1nx/EventEmitter2
   *
   * Copyright (c) 2013 hij1nx
   * Licensed under the MIT license.
   *)
*/
//# sourceMappingURL=roslib.js.map
