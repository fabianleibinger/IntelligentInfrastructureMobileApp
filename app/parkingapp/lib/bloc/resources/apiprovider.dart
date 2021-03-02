import 'dart:async';
import 'package:http/http.dart' show Client, Response;
import 'dart:convert';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/classes/vehicle.dart';

// defines the calls to the backend and what should happen with the results

class ApiProvider {
  static final int statusCodeSuccess = 200;
  static final String hostname = 'fzi-autopark-backend.local';
  static final int serverPort = 2525;

  static Future<Map<String, dynamic>> getWelcome() async {
    return httpGet('http://$hostname:$serverPort/', 'Failed to get welcomed');
  }

  static Future<Map<String, dynamic>> connect() async {
    return httpGet(
        'http://$hostname:$serverPort/connect', 'Failed to connect to server');
  }

  //http get request that returns the response body
  static Future<Map<String, dynamic>> httpGet(
      String url, String failureText) async {
    Client client = Client();
    final Response response = await client.get(url);
    final Map result = json.decode(response.body);
    if (response.statusCode == statusCodeSuccess) {
      return result;
    } else {
      throw Exception(failureText);
    }
  }

  //
  static Future<Map<String, dynamic>> parkIn(Vehicle vehicle) async {
    Client client = Client();
    final response = await client.post("http://$hostname:$serverPort/",
        headers: {"Authorization": vehicle.inAppKey},
        body: jsonEncode({
          "nearExitPreference": vehicle.nearExitPreference,
          "parkingCard": vehicle.parkingCard
        }));
    final Map result = json.decode(response.body);
    if (response.statusCode == 201) {
      return result;
    } else {
      throw Exception('Failed to load post');
    }
  }

  static Future<Map<String, dynamic>> parkOut(Vehicle vehicle) async {
    Client client = Client();
    final response = await client.get("http://$hostname:$serverPort/",
        headers: {"Authorization": vehicle.inAppKey});
    final Map result = json.decode(response.body);
    if (response.statusCode == 201) {
      return result;
    } else {
      throw Exception('Failed to load post');
    }
  }

  static Future<Map<String, dynamic>> getFreeParkingSpots(
      Vehicle vehicle) async {
    Client client = Client();
    final response = await client.get("http://$hostname:$serverPort/",
        headers: {"Authorization": vehicle.inAppKey});
    final Map result = json.decode(response.body);
    if (response.statusCode == 201) {
      return result;
    } else {
      throw Exception('Failed to load post');
    }
  }

  static Future<Map<String, dynamic>> getPosition(Vehicle vehicle) async {
    Client client = Client();
    final response = await client.get("http://$hostname:$serverPort/",
        headers: {"Authorization": vehicle.inAppKey});
    final Map result = json.decode(response.body);
    if (response.statusCode == 201) {
      return result;
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
