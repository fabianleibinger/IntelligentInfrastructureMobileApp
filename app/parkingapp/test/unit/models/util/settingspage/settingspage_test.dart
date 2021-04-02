import 'package:flutter_test/flutter_test.dart';
import 'package:parkingapp/ui/settingspage/passcodepage.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();
  PasscodePageState _passcodePageState;
  bool _passcodeEnabled;
  SharedPreferences _prefs;
  setUp(() async {
    SharedPreferences.setMockInitialValues({});
    _prefs = await SharedPreferences.getInstance();
    _passcodeEnabled = true;
    _passcodePageState = PasscodePageState();
  });
  test('Passcode should be disabled', () {
    _passcodePageState.resetAppPassword();
    _prefs.setBool('authentification', false);
    _passcodeEnabled = _prefs.getBool('authentification') ?? true;
    expect(_passcodeEnabled, false);
  });

  tearDown(() {
    _passcodePageState = null;
    _prefs = null;
    _passcodeEnabled = null;
  });
}
