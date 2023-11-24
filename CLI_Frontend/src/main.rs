use libc::c_int;

#[link(name = "JS_dynlib")]
extern {
    fn test(input: c_int) -> c_int;
}

fn main() {
    println!("Hello, Rust!");

    let x = unsafe { test(10) };

    println!("the result is: {}", x);
}
