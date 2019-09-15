export TRAVIS_BUILD_DIR=${PWD}
export PATH="${TRAVIS_BUILD_DIR}/buildroot/bin/:${TRAVIS_BUILD_DIR}/buildroot/share/tests/:${PATH}"
wwtd -u before_install,install,before_script
