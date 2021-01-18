import 'package:shared_preferences/shared_preferences.dart';

class User {
  String username;

  User(this.username);

  static User fromJson(Map<String, dynamic> parsedJson) {
    return new User(parsedJson['username']);
  }

  Future setUsername(String username) async {
    final prefs = await SharedPreferences.getInstance();
    prefs.setString('username', username);
  }

  Future<String> getUsername() async {
    final prefs = await SharedPreferences.getInstance();
    return prefs.getString('username');
  }
}
