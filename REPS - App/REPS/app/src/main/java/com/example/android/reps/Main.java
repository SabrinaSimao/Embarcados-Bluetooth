package com.example.android.reps;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;

/**
 * Created by gabrielaalmeida on 31/05/17.
 */

public class Main extends Activity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
    }

    public void buttonDoOnClick(View view) {
        final Intent do_exercise = new Intent(this, ExerciseInfo.class);
        startActivity(do_exercise);
    }

    public void buttonLearnOnClick(View view){
        final Intent learn_exercise = new Intent(this, Main.class);
        startActivity(learn_exercise);
    }
}