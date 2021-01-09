// Example for a class which is used in the app (here "user")

class User {
  int id;
  String apikey;
  String username;
  String password;

  User(this.id, this.apikey, this.username, this.password);

  static User fromJson(Map<String, dynamic> parsedJson) {
    return new User(parsedJson['id'], parsedJson['apikey'],
        parsedJson['username'], parsedJson['password']);
  }
}
