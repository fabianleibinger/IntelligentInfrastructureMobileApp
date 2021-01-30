import 'package:uuid/uuid.dart';

class Utility {
  static generateKey() {
    var uuid = Uuid();
    return uuid.v1();
  }
}
