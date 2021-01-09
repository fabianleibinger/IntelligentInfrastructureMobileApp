import 'package:flutter/material.dart';

import '../global.dart';

// With such a class u can define a widget that can be used as a list element

class IntrayTodo extends StatelessWidget {
  final String keyValue;
  final String title;
  final String note;
  IntrayTodo({this.keyValue, this.title, this.note});

  @override
  Widget build(BuildContext context) {
    return Container(
      key: Key(keyValue),
      margin: EdgeInsets.only(left: 15, right: 15, bottom: 15),
      padding: EdgeInsets.all(10),
      height: 100,
      decoration: BoxDecoration(
        borderRadius: BorderRadius.all(Radius.circular(10)),
        boxShadow: [
          new BoxShadow(
            color: Colors.black.withOpacity(0.5),
            blurRadius: 10.0,
          )
        ],
      ),
      child: Row(
        children: <Widget>[
          Column(
            children: <Widget>[
              Text(
                title,
              ),
              Text(note)
            ],
          )
        ],
      ),
    );
  }
}
