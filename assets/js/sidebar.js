let button = document.querySelector("#button-menu");
let menu = document.querySelector("menu");

button.addEventListener("click", () => {
  menu.classList.toggle("sidebar")
})