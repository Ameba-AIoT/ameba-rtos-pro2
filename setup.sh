#!/bin/bash

cp .git_hooks/* .git/hooks
chmod a+x .git/hooks/*

echo "Git Hook setup complete"
