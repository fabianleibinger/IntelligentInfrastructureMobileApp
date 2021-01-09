import '../resources/repository.dart';
import 'package:rxdart/rxdart.dart';
import 'package:parkingapp/models/classes/user.dart';

// by UserBloc.'method' you can call one of the methods from below.
// These are defined in api.dart

final userBloc = UserBloc();

class UserBloc {
  final _repository = Repository();
  final _userGetter = PublishSubject<User>();

  Stream<User> get getUser => _userGetter.stream;

  Future<User> registerUser(String username) async {
    User user = await _repository.registerUser(username);
    _userGetter.sink.add(user);
    return user;
  }

  Future<User> signinUser(String username, String apikey) async {
    User user = await _repository.signinUser(username, apikey);
    _userGetter.sink.add(user);
    return user;
  }

  dispose() {
    _userGetter.close();
  }
}
