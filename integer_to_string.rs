/* Problem: Convert a number into a string. */

fn output<'a>(input: i64) -> String {
    const DIGIT: &'static [&'static str] = &[
        "", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine",
    ];
    const SPECIAL: &'static [&'static str] = &[
        "ten",
        "eleven",
        "twelve",
        "thirteen",
        "fourteen",
        "fifteen",
        "sixteen",
        "seventeen",
        "eighteen",
        "nineteen",
    ];
    let tens = [
        "", "", "twenty", "thirty", "fourty", "fifty", "sixty", "seventy", "eighty", "ninety",
    ];
    let groups = [
        "",
        "thousand",
        "million",
        "billion",
        "trillion",
        "quadrillion",
        "quintillion",
        "sextillion",
    ];
    fn digit_with_special(tens: u8, ones: u8) -> &'static str {
        if tens == 1 {
            SPECIAL[ones as usize]
        } else {
            DIGIT[ones as usize]
        }
    }
    struct State {
        current: i64,
    }
    impl Iterator for State {
        type Item = u8;

        fn next(&mut self) -> Option<Self::Item> {
            if self.current == 0 {
                None
            } else {
                let n = self.current % 10;
                self.current = self.current / 10;
                Some(n as u8)
            }
        }
    }
    if (input == 0) {
        "zero".to_string()
    } else {
        format!(
            "{}{}",
            if input < 0 { "negative " } else { "" },
            State {
                current: input.abs()
            }
            .collect::<Vec<u8>>()
            .chunks(3)
            .enumerate()
            .map(|(i, group)| {
                if group == [0, 0, 0] {
                    vec![]
                } else {
                    if group.len() == 1 {
                        vec![DIGIT[group[0] as usize], groups[i]]
                    } else if group.len() == 3 && group[2] != 0 {
                        vec![
                            DIGIT[group[2] as usize],
                            "hundred",
                            tens[group[1] as usize],
                            digit_with_special(group[1], group[0]),
                            groups[i],
                        ]
                    } else {
                        vec![
                            tens[group[1] as usize],
                            digit_with_special(group[1], group[0]),
                            groups[i],
                        ]
                    }
                }
            })
            .rev()
            .flatten()
            .filter(|s| !s.is_empty())
            .collect::<Vec<&str>>()
            .join(" ")
        )
        .to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero() {
        assert_eq!(output(0), "zero")
    }

    #[test]
    fn twelve() {
        assert_eq!(output(12), "twelve")
    }

    #[test]
    fn twenty_one() {
        assert_eq!(output(21), "twenty one")
    }

    #[test]
    fn five_hundred() {
        assert_eq!(output(500), "five hundred")
    }

    #[test]
    fn _12_345() {
        assert_eq!(output(12_345), "twelve thousand three hundred fourty five")
    }

    #[test]
    fn _1_234_567_890() {
        assert_eq!(
            output(1_234_567_890),
            "one billion two hundred thirty four million five hundred sixty seven thousand eight hundred ninety"
        )
    }

    #[test]
    fn five_million() {
        assert_eq!(output(5_000_000), "five million")
    }

    #[test]
    fn five_million_one() {
        assert_eq!(output(5_000_001), "five million one")
    }

    #[test]
    fn neg_five_million() {
        assert_eq!(output(-5_000_000), "negative five million")
    }
}
