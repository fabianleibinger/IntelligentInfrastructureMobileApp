import 'dart:async';
import 'apiprovider.dart';
import 'package:parkingapp/models/classes/user.dart';

// connector between user_blocs_provider.dart and api.dart

class Repository {
  final apiProvider = ApiProvider();

  Future<User> registerUser(String username) =>
      apiProvider.registerUser(username);

  Future<User> signinUser(String username, String apikey) =>
      apiProvider.signinUser(username, apikey);
}
