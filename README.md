# RoboTune

A browser-only characterization and PIDF autotuning prototype for VEX, FTC, and FRC.

## GitHub Pages deployment

1. Create a new GitHub repository, for example `robotune`.
2. Upload **all files in this folder** to the repository root.
3. Open **Settings → Pages**.
4. Under **Build and deployment**, choose **Deploy from a branch**.
5. Select the `main` branch and `/ (root)`.
6. Click **Save**.
7. Wait for the GitHub Actions/Pages deployment to finish.
8. Open the generated `https://USERNAME.github.io/robotune/` URL.

The application runs entirely in the browser. Uploaded JSON characterization files are processed locally and are not sent to a server.

## JSON format

```json
{
  "metadata": {
    "platform": "VEX",
    "mechanism": "linear_slide"
  },
  "samples": [
    {
      "time": 0.000,
      "voltage": 0.0,
      "position": 0.0,
      "velocity": 0.0
    }
  ]
}
```
