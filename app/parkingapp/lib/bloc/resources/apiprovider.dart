import 'dart:async';
import 'package:http/http.dart' show Client, Response;
import 'dart:convert';
import 'package:parkingapp/models/classes/user.dart';

// defines the calls to the backend and what should happen with the results

class ApiProvider {
  static Future<String> getWelcome() async {
    Client client = Client();
    final response = await client.get("http://192.168.178.100:5000/");
    final Map result = json.decode(response.body);
    if (response.statusCode == 201) {
      return result["message"];
    } else {
      throw Exception('Failed to load post');
    }
  }

  /*Future<User> signinUser(String username, String apikey) async {
    final response = await client.post("http://192.168.178.48:5000/api/signin",
        headers: {"Authorization": apikey},
        body: jsonEncode({
          "username": username,
        }));
    final Map result = json.decode(response.body);
    if (response.statusCode == 201) {
      // If the call to the server was successful, parse the JSON
      return User.fromJson(result["data"]);
    } else {
      // If that call was not successful, throw an error.
      throw Exception('Failed to load post');
    }
  }*/
}
