import 'package:flutter/material.dart';
import 'package:parkingapp/bloc/blocs/user_bloc_provider.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:parkingapp/models/classes/user.dart';
import 'package:parkingapp/models/global.dart';

// example for a page (loginpage)

class LoginPage extends StatefulWidget {
  final VoidCallback login;

  const LoginPage({Key key, this.login}) : super(key: key);
  @override
  _LoginPageState createState() => _LoginPageState();
}

class _LoginPageState extends State<LoginPage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.grey,
      body: Center(
        child: getSigninPage(),
      ),
    );
  }

  Widget getSigninPage() {
    TextEditingController usernameController = new TextEditingController();
    return Container(
      margin: EdgeInsets.only(top: 100, left: 20, right: 20, bottom: 100),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: <Widget>[
          Text(
            "Welcome!",
          ),
          Container(
            height: 200,
            child: Column(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: <Widget>[
                Theme(
                  data: Theme.of(context)
                      .copyWith(splashColor: Colors.transparent),
                  child: TextField(
                    controller: usernameController,
                    autofocus: false,
                    style: TextStyle(fontSize: 22.0, color: Colors.grey),
                    decoration: InputDecoration(
                        filled: true,
                        fillColor: Colors.white,
                        hintText: "Username",
                        contentPadding: const EdgeInsets.only(
                            left: 14.0, bottom: 8.0, top: 8.0),
                        focusedBorder: OutlineInputBorder(
                            borderSide: BorderSide(color: Colors.white),
                            borderRadius: BorderRadius.circular(25.7)),
                        enabledBorder: UnderlineInputBorder(
                            borderSide: BorderSide(color: Colors.white),
                            borderRadius: BorderRadius.circular(25.7))),
                  ),
                ),
                FlatButton(
                  child: Text("Sign in"),
                  onPressed: () {
                    if (usernameController.text != "") {
                      userBloc
                          .signinUser(usernameController.text, "")
                          .then((user) {
                        if (user != null) {
                          saveApiKey(user);
                          widget.login();
                        }
                      });
                    }
                  },
                )
              ],
            ),
          ),
          Container(
            child: Column(
              children: <Widget>[
                Text(
                  "Don't you have an account yet?!",
                ),
                FlatButton(
                    child: Text(
                      "create one",
                    ),
                    onPressed: () {})
              ],
            ),
          )
        ],
      ),
    );
  }

  void saveApiKey(User user) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('API_Token', user.apikey);
  }
}
