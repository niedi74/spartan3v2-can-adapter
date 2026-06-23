package com.spartan.hubdisplay;

import android.app.Activity;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.os.Bundle;
import android.view.Gravity;
import android.view.KeyEvent;
import android.view.View;
import android.view.ViewGroup.LayoutParams;
import android.view.WindowManager;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;

/**
 * Spartan Hub Display: minimaler Vollbild-WebView, der die Hub-Oberflaeche laedt
 * und den Bildschirm dauerhaft anlaesst (wie die 123TUNE+ App). URL ist per
 * Eingabefeld + Presets (AP / Heimnetz) frei waehlbar und wird gemerkt.
 */
public class MainActivity extends Activity {

    private static final String PREFS = "hub";
    private static final String KEY_URL = "url";
    private static final String AP_URL = "http://192.168.4.1/";
    private static final String HOME_URL = "http://192.168.0.91/";

    private WebView web;
    private LinearLayout configView;
    private EditText urlInput;
    private SharedPreferences prefs;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        prefs = getSharedPreferences(PREFS, Context.MODE_PRIVATE);

        web = new WebView(this);
        web.setKeepScreenOn(true);
        WebSettings s = web.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);
        s.setUseWideViewPort(true);
        s.setLoadWithOverviewMode(true);
        s.setCacheMode(WebSettings.LOAD_DEFAULT);
        web.setWebViewClient(new WebViewClient() {
            @Override
            public boolean shouldOverrideUrlLoading(WebView v, String url) {
                v.loadUrl(url);
                return true;
            }
        });

        buildConfigView();

        String saved = prefs.getString(KEY_URL, null);
        if (saved != null && saved.length() > 0) {
            connect(saved);
        } else {
            showConfig();
        }
    }

    private void buildConfigView() {
        configView = new LinearLayout(this);
        configView.setOrientation(LinearLayout.VERTICAL);
        configView.setGravity(Gravity.CENTER);
        configView.setBackgroundColor(Color.parseColor("#111111"));
        configView.setPadding(48, 48, 48, 48);

        TextView title = new TextView(this);
        title.setText("Spartan Hub Display");
        title.setTextColor(Color.parseColor("#e8e8e8"));
        title.setTextSize(22);
        title.setGravity(Gravity.CENTER);
        title.setPadding(0, 0, 0, 32);
        configView.addView(title);

        urlInput = new EditText(this);
        urlInput.setText(prefs.getString(KEY_URL, AP_URL));
        urlInput.setTextColor(Color.WHITE);
        urlInput.setHint("http://...");
        urlInput.setSingleLine(true);
        LinearLayout.LayoutParams urlLp =
                new LinearLayout.LayoutParams(900, LayoutParams.WRAP_CONTENT);
        configView.addView(urlInput, urlLp);

        Button connect = mkButton("Verbinden", "#d2691e");
        connect.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                connect(normalize(urlInput.getText().toString()));
            }
        });
        configView.addView(connect);

        LinearLayout presets = new LinearLayout(this);
        presets.setOrientation(LinearLayout.HORIZONTAL);
        presets.setGravity(Gravity.CENTER);
        presets.setPadding(0, 24, 0, 0);

        Button ap = mkButton("AP 192.168.4.1", "#2a4d2a");
        ap.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                urlInput.setText(AP_URL);
            }
        });
        Button home = mkButton("Heim .0.91", "#2a3d5d");
        home.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                urlInput.setText(HOME_URL);
            }
        });
        presets.addView(ap);
        presets.addView(home);
        configView.addView(presets);

        TextView hint = new TextView(this);
        hint.setText("Bildschirm bleibt an, solange die App offen ist. "
                + "Zurueck-Taste zeigt diese Seite wieder.");
        hint.setTextColor(Color.parseColor("#888888"));
        hint.setTextSize(12);
        hint.setGravity(Gravity.CENTER);
        hint.setPadding(0, 32, 0, 0);
        configView.addView(hint);
    }

    private Button mkButton(String text, String color) {
        Button b = new Button(this);
        b.setText(text);
        b.setTextColor(Color.WHITE);
        b.setBackgroundColor(Color.parseColor(color));
        LinearLayout.LayoutParams lp = new LinearLayout.LayoutParams(
                LayoutParams.WRAP_CONTENT, LayoutParams.WRAP_CONTENT);
        lp.setMargins(12, 12, 12, 12);
        b.setLayoutParams(lp);
        return b;
    }

    private String normalize(String u) {
        u = u.trim();
        if (u.length() == 0) {
            u = AP_URL;
        }
        if (!u.startsWith("http://") && !u.startsWith("https://")) {
            u = "http://" + u;
        }
        return u;
    }

    private void connect(String url) {
        prefs.edit().putString(KEY_URL, url).apply();
        setContentView(web);
        enableImmersive();
        web.loadUrl(url);
    }

    private void showConfig() {
        setContentView(configView);
    }

    private void enableImmersive() {
        web.setSystemUiVisibility(
                View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                        | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                        | View.SYSTEM_UI_FLAG_FULLSCREEN
                        | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (web.getParent() != null) {
            enableImmersive();
        }
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (keyCode == KeyEvent.KEYCODE_BACK && web.getParent() != null) {
            if (web.canGoBack()) {
                web.goBack();
            } else {
                showConfig();
            }
            return true;
        }
        return super.onKeyDown(keyCode, event);
    }
}
