import * as params from '@params';

document
    .getElementById("quiz")
    .addEventListener("submit", (event) => {
        event.preventDefault();
        console.log("test");
        return false;
});