import 'dart:async';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'package:parkingapp/models/classes/vehicle.dart';

// defines the calls to the backend and what should happen with the results

class ApiProvider {
  static final int httpGetStatusCodeSuccess = 200;
  static final int httpPostStatusCodeSuccess = 201;

  // ['IP', 'Port', 'Parking garage']
  //tries to connect to server
  static Future<Map<String, dynamic>> connect() async {
    return httpGet(
        'http://10.0.2.2:2525/connect', 'Failed to connect to server');
  }

  // ['total', 'electric', 'electric_fast', 'electric_inductive']
  //tries to get parking capacities
  static Future<Map<String, dynamic>> getCapacity() async {
    return httpGet(
        'http://10.0.2.2:2525/capacities', 'Failed to get capacities');
  }

  // ['free_total', 'free_electric', 'free_electric_fast', 'free_electric_inductive']
  //tries to get free parking capacities
  static Future<Map<String, dynamic>> getFreeCapacity() async {
    return httpGet('http://10.0.2.2:2525/free', 'Failed to get capacities');
  }

  // ['free_total']
  //tries to get free parking spots
  static Future<Map<String, dynamic>> getFreeParkingSpots() async {
    return httpGet(
        'http://10.0.2.2:2525/free/total', 'Failed to get free parking spots');
  }

  // ['free_electric']
  //tries to get free chargeable parking spots
  static Future<Map<String, dynamic>> getFreeChargeableParkingSpots() async {
    return httpGet('http://10.0.2.2:2525/free/electric',
        'Failed to get free chargeable parking spots');
  }

  //http get request that returns the response body
  static Future<Map<String, dynamic>> httpGet(
      String url, String failureText) async {
    final response = await http.get(url);
    if (response.statusCode == httpGetStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      print(result.entries.toString());
      return result;
    } else {
      throw Exception(failureText);
    }
  }

  //tries to park in vehicle
  static Future<Map<String, dynamic>> parkIn(Vehicle vehicle) async {
    final response = await http.post("http://10.0.2.2:2525/parkIn",
        headers: {"Authorization": vehicle.inAppKey},
        body: jsonEncode({
          "nearExitPreference": vehicle.nearExitPreference,
          "parkingCard": vehicle.parkingCard
        }));
    if (response.statusCode == httpPostStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      return result;
    } else {
      throw Exception('Failed to park in');
    }
  }

  //tries to park out vehicle or cancel park in
  static Future<Map<String, dynamic>> parkOut(Vehicle vehicle) async {
    final response = await http.get("http://10.0.2.2:2525/parkOut",
        headers: {"Authorization": vehicle.inAppKey});
    if (response.statusCode == httpPostStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      return result;
    } else {
      throw Exception('Failed to park out');
    }
  }

  //tries to get position of the vehicle
  static Future<Map<String, dynamic>> getPosition(Vehicle vehicle) async {
    final response = await http.get("http://10.0.2.2:2525/getPosition",
        headers: {"Authorization": vehicle.inAppKey});
    if (response.statusCode == httpPostStatusCodeSuccess) {
      final Map result = json.decode(response.body);
      return result;
    } else {
      throw Exception('Failed to get position');
    }
  }
}
