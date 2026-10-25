#!/bin/bash
# 日本語パスだとAGPがビルドできないため、ASCIIパスへ同期してビルドする
export JAVA_HOME="/c/Program Files/Microsoft/jdk-17.0.20.101-hotspot"
export PATH="$JAVA_HOME/bin:/c/Android/gradle/gradle-8.7/bin:$PATH"
SRC="$(cd "$(dirname "$0")" && pwd)"
DST=/c/Android/work/rcj-vision
mkdir -p "$DST"
cp -r "$SRC"/app/src "$DST"/app/ 2>/dev/null || { mkdir -p "$DST/app" && cp -r "$SRC"/app/src "$DST"/app/; }
cp "$SRC"/app/build.gradle.kts "$DST"/app/
cp "$SRC"/build.gradle.kts "$SRC"/settings.gradle.kts "$SRC"/gradle.properties "$SRC"/local.properties "$DST"/
cd "$DST" && gradle assembleDebug --console=plain "$@" && cp app/build/outputs/apk/debug/app-debug.apk "$SRC"/rcj-vision-debug.apk
