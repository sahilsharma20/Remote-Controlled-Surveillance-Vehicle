# Upload This Professional Version to GitHub

The package in this folder is ready to replace the contents of the existing repository. Your Git history should remain intact.

## Recommended macOS/Linux method

```bash
# 1. Clone the existing repository into a fresh working folder.
git clone https://github.com/sahilsharma20/Remote-Controlled-Surveillance-Vehicle.git rover-repo

# 2. Copy this professional package over it.
# Replace /path/to/... with the location of the extracted package.
rsync -av --delete --exclude='.git' \
  /path/to/Remote-Controlled-Surveillance-Vehicle-professional/ \
  rover-repo/

# 3. Review and publish.
cd rover-repo
git status
git add -A
git commit -m "refactor: professionalize vision-guided rover project"
git push origin main
```

`rsync --delete` removes old repository files that are not part of the professional package. Confirm both paths carefully before running it.

## After pushing

1. Open the GitHub repository and select the gear icon next to **About**.
2. Add the description and topics from `docs/REPOSITORY_SETUP.md`.
3. Confirm that the CI workflow passes under **Actions**.
4. Open the README and verify that the image, GIF and MP4 link work.
5. Run the dry-run command locally before testing the Arduino.
