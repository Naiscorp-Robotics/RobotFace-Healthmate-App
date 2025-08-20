import QtQuick 2.12
import QtQuick.Controls 2.12
import QtMultimedia 5.12

// An optimized video player for embedded systems
// Uses GStreamer backend for better performance
Item {
    id: root
    
    // Public properties
    property string source: "qrc:/assets/blinking_face.mp4"
    property bool autoPlay: false
    property bool playing: videoPlayer.playbackState === MediaPlayer.PlayingState
    property int loops: 1
    property int currentLoop: 0
    
    // Signals
    signal started()
    signal stopped()
    signal error(string message)
    
    // Video display with GStreamer backend
    Video {
        id: videoPlayer
        anchors.fill: parent
        source: root.source
        fillMode: VideoOutput.PreserveAspectFit
        autoPlay: false
        loops: root.loops === -1 ? MediaPlayer.Infinite : root.loops
        
        // Error handling
        onErrorChanged: {
            if (error !== MediaPlayer.NoError) {
                console.error("Video error:", errorString);
                root.error(errorString);
            }
        }
        
        // Status monitoring
        onStatusChanged: {
            console.log("Video status changed:", status);
            if (status === MediaPlayer.Loaded) {
                console.log("Video loaded successfully");
            }
        }
        
        // Playback state monitoring
        onPlaybackStateChanged: {
            if (playbackState === MediaPlayer.PlayingState) {
                root.started();
            } else if (playbackState === MediaPlayer.StoppedState) {
                root.stopped();
            }
        }
    }
    
    // Debug info (hidden)
    Text {
        anchors {
            top: parent.top
            left: parent.left
            margins: 10
        }
        color: "white"
        font.pixelSize: 10
        text: "Video Status: " + videoPlayer.status +
              "\nError: " + videoPlayer.error +
              "\nPlaying: " + (videoPlayer.playbackState === MediaPlayer.PlayingState ? "Yes" : "No") +
              "\nDuration: " + Math.round(videoPlayer.duration/1000) + "s" +
              "\nSource: " + videoPlayer.source
        visible: false // Set to true for debugging
        z: 100
    }
    
    Component.onCompleted: {
        console.log("Optimized video player loaded, source:", source);
        
        if (autoPlay) {
            // Delay playback to ensure proper initialization
            playTimer.start();
        }
    }
    
    // Timer for delayed playback
    Timer {
        id: playTimer
        interval: 500
        repeat: false
        onTriggered: {
            console.log("Starting video playback after delay");
            root.play();
        }
    }
    
    // Public methods
    function play() {
        videoPlayer.play();
    }
    
    function pause() {
        videoPlayer.pause();
    }
    
    function stop() {
        videoPlayer.stop();
    }
    
    function restart() {
        videoPlayer.seek(0);
        videoPlayer.play();
    }
}
