import 'package:flutter/material.dart';
import 'package:parkingapp/ui/FirstStart/appconfiguration.dart';
import 'package:flutter_gen/gen_l10n/app_localizations.dart';

class TermsOfService extends StatefulWidget {
  @override
  _TermsOfServiceState createState() => _TermsOfServiceState();
}

class _TermsOfServiceState extends State<TermsOfService> {
  final _formKey = GlobalKey<FormState>();
  bool _switch = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(AppLocalizations.of(context).titleWelcome),
      ),
      bottomNavigationBar: Material(
        elevation: 10,
        child: ButtonBar(
          children: [
            TextButton(
              child: Text(AppLocalizations.of(context).buttonContinue),
              onPressed: () => _formKey.currentState.validate()
                  ? Navigator.push(
                      context,
                      MaterialPageRoute(
                          builder: (context) => AppConfiguration()))
                  : null,
            )
          ],
        ),
      ),
      body: Column(
        children: [
          //TOS text
          Expanded(
              child: ListView(
                  children: [Text(AppLocalizations.of(context).terms)])),
          //accept terms checkbox
          Form(
            key: _formKey,
            child: FormField(
              builder: (FormFieldState<dynamic> field) {
                return CheckboxListTile(
                  title: Text(AppLocalizations.of(context).acceptTerms),
                  //show that this field is required if in an incorrect state
                  subtitle: field.hasError
                      ? Text(
                          field.errorText,
                          style: TextStyle(color: Theme.of(context).errorColor),
                        )
                      : null,
                  controlAffinity: ListTileControlAffinity.leading,
                  onChanged: (bool value) {
                    setState(() {
                      _switch = value;
                      _formKey.currentState.validate();
                    });
                  },
                  value: _switch,
                );
              },
              //validate if the state is in the correct position
              //if in off position return the error text
              validator: (value) =>
                  !_switch ? AppLocalizations.of(context).requiredText : null,
            ),
          ),
        ],
      ),
    );
  }
}
