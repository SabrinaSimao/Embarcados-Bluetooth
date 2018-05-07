package com.example.android.reps;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;

/**
 * Created by gabrielaalmeida on 31/05/17.
 */

public class ExerciseInfo extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.learn);
    }

    public void learn(View view) {
        final Intent learn_ex = new Intent(this, LearExercise.class);
        startActivity(learn_ex);
    }

}
